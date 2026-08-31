/****************************************************************************
 * flight_mode_bridge: MPA <-> PX4 external-flight-modes bridge.
 *
 * Serves the "px4_modes" MPA pipe. Protocol: voxl_px4_modes_pipe.h (a
 * synced copy — the original lives in the apps-side example repo, currently
 * voxl-figure-eight; edit there and copy here). Translates pipe messages
 * from apps-side mode clients into PX4's external-component uORB contract
 * and back. Supports up to 8 concurrent clients (one mode each), routed
 * by request_id.
 *
 *   client -> bridge (control pipe): register, health, setpoints, unregister
 *   bridge -> client (data pipe):    register reply, mode (de)activation,
 *                                    vehicle state broadcast (~50 Hz)
 *
 * Setpoint payloads are standard MAVLink offboard structs
 * (SET_POSITION_TARGET_LOCAL_NED / SET_ATTITUDE_TARGET); translation logic
 * mirrors mavlink_receiver.cpp. Each mode declares a setpoint type at
 * registration; the bridge forwards it via SetpointConfig so commander
 * derives the control flags and arms a setpoint-timeout failsafe.
 *
 * Design notes:
 *  - Arming-check polls are answered FROM CACHE (client streams health at
 *    ~1 Hz; any client message refreshes aliveness; >3 s stale => can_arm
 *    false so PX4 refuses/falls out of the mode on a dead client).
 *  - MPA control callback runs on the MPA thread: shared state is mutexed;
 *    uORB publish from that thread follows the crsf_bridge precedent.
 ****************************************************************************/

#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/setpoint_config_reply.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <pthread.h>
#include <string.h>
#include <math.h>

#include "mpa.hpp"
#include "voxl_px4_modes_pipe.h"
#include "voxl_px4_modes_mavlink_structs.h"

using namespace time_literals;

static constexpr uint64_t HEALTH_STALE_US = 3_s;
static constexpr uint16_t SETPOINT_TIMEOUT_MS = 500;

class FlightModeBridge : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	FlightModeBridge() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}
	~FlightModeBridge() override { cleanup(); }

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init()
	{
		if (MPA::Initialize() < 0) {
			PX4_ERR("MPA init failed");
			return false;
		}

		char pipe_name[] = VOXL_PX4_MODES_PIPE_NAME;
		_pipe_ch = MPA::PipeCreate(pipe_name, SERVER_FLAG_EN_CONTROL_PIPE);

		if (_pipe_ch < 0) {
			PX4_ERR("pipe create failed");
			return false;
		}

		if (MPA::PipeServerSetControlCb(_pipe_ch, &FlightModeBridge::control_callback, this) < 0) {
			PX4_ERR("control cb setup failed");
			cleanup();
			return false;
		}

		ScheduleOnInterval(20_ms);
		PX4_INFO("serving pipe '%s' (protocol v%d)", pipe_name, VOXL_PX4_MODES_PROTOCOL_VERSION);
		return true;
	}

	void cleanup()
	{
		ScheduleClear();

		if (_pipe_ch >= 0) {
			MPA::PipeServerClose(_pipe_ch);
			_pipe_ch = -1;
		}
	}

	void request_stop() override
	{
		ModuleBase::request_stop();
		ScheduleNow();
	}

private:
	// client registry: one entry per registered mode, keyed by request_id
	struct Client {
		bool in_use{false};
		uint64_t request_id{0};
		char name[VOXL_PX4_MODES_NAME_LEN] {};
		uint8_t setpoint_type{MODE_SETPOINT_TRAJECTORY};
		bool px4_ack{false};          // commander replied success
		int8_t mode_id{-1};
		int8_t arming_check_id{-1};
		bool can_arm{false};
		uint32_t reqs{0};
		hrt_abstime last_seen{0};
		bool active{false};
		hrt_abstime sp_config_sent{0};
		hrt_abstime reg_forwarded{0};
	};

	// Declare a mode's setpoint type to commander. Commander resets the
	// stored type back to the default (Trajectory) whenever a mode
	// deactivates (see ModeManagement::getSetpointType), so this must be
	// re-sent on every activation. With a stale default, commander enables
	// the wrong controller set for the mode and PX4's own controllers will
	// fight the mode's setpoints on the shared setpoint topics.
	void publish_setpoint_config(Client &c)
	{
		setpoint_config_s sc{};
		sc.timestamp = hrt_absolute_time();
		sc.type = c.setpoint_type;
		sc.source_id = (uint8_t)c.mode_id;
		sc.should_apply = true;
		sc.timeout_ms = SETPOINT_TIMEOUT_MS;
		_sp_config_pub.publish(sc);
		c.sp_config_sent = sc.timestamp;
	}

	void Run() override
	{
		if (should_exit()) {
			cleanup();
			exit_and_cleanup(desc);
			return;
		}

		pthread_mutex_lock(&_mutex);

		// 1) registration replies from commander -> forward to client,
		//    then declare the mode's setpoint type (SetpointConfig)
		register_ext_component_reply_s rep;

		while (_reply_sub.update(&rep)) {
			Client *c = find_client(rep.request_id);

			if (c == nullptr) { continue; }

			c->px4_ack = rep.success;
			c->mode_id = rep.mode_id;
			c->arming_check_id = rep.arming_check_id;

			mode_register_reply_t out{};
			out.request_id = c->request_id;
			out.success = rep.success;
			out.mode_id = rep.mode_id;
			out.arming_check_id = rep.arming_check_id;
			pipe_send(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
			PX4_INFO("mode '%s' registered: mode_id=%d", c->name, rep.mode_id);

			if (rep.success) {
				publish_setpoint_config(*c);
			}
		}

		// 1b) setpoint-config replies: log the verdict
		setpoint_config_reply_s scr;

		while (_sp_config_reply_sub.update(&scr)) {
			for (int i = 0; i < MAX_CLIENTS; i++) {
				if (_clients[i].in_use && _clients[i].mode_id == (int8_t)scr.source_id) {
					if (scr.result == setpoint_config_reply_s::RESULT_SUCCESS) {
						PX4_INFO("mode '%s' setpoint type %d accepted", _clients[i].name, scr.type);

					} else {
						PX4_ERR("mode '%s' setpoint type %d REJECTED (result=%d)",
							_clients[i].name, scr.type, scr.result);
					}
				}
			}
		}

		// 2) arming-check polls -> answer from cache, once per client
		arming_check_request_s ac_req;

		while (_ac_req_sub.update(&ac_req)) {
			for (int i = 0; i < MAX_CLIENTS; i++) {
				Client &cl = _clients[i];

				if (!cl.in_use || !cl.px4_ack) { continue; }

				const bool fresh = (hrt_absolute_time() - cl.last_seen) < HEALTH_STALE_US;

				arming_check_reply_s ac{};
				ac.timestamp = hrt_absolute_time();
				ac.request_id = ac_req.request_id;
				ac.registration_id = (uint8_t)cl.arming_check_id;
				ac.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
				ac.can_arm_and_run = cl.can_arm && fresh;
				ac.mode_req_angular_velocity = cl.reqs & MODE_REQ_ANGULAR_VELOCITY;
				ac.mode_req_attitude = cl.reqs & MODE_REQ_ATTITUDE;
				ac.mode_req_local_alt = cl.reqs & MODE_REQ_LOCAL_ALT;
				ac.mode_req_local_position = cl.reqs & MODE_REQ_LOCAL_POSITION;
				ac.mode_req_global_position = cl.reqs & MODE_REQ_GLOBAL_POSITION;
				ac.mode_req_manual_control = cl.reqs & MODE_REQ_MANUAL_CONTROL;
				_ac_reply_pub.publish(ac);
			}
		}

		// 3) activation watching: nav_state enters/leaves a client's slot
		vehicle_status_s vs;

		if (_vstatus_sub.update(&vs)) {
			_armed = (vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_nav_state = vs.nav_state;

			for (int i = 0; i < MAX_CLIENTS; i++) {
				Client &cl = _clients[i];

				if (!cl.in_use || !cl.px4_ack) { continue; }

				const bool now_active = (vs.nav_state == (uint8_t)cl.mode_id);

				if (now_active != cl.active) {
					cl.active = now_active;

					if (now_active) {
						// commander wiped the type on last deactivation; re-declare
						publish_setpoint_config(cl);
					}

					vehicle_local_position_s lp{};
					_lpos_sub.copy(&lp);

					mode_state_t st{};
					st.request_id = cl.request_id;
					st.active = now_active;
					st.armed = _armed;
					st.position[0] = lp.x; st.position[1] = lp.y; st.position[2] = lp.z;
					st.yaw = lp.heading;
					pipe_send(MODE_MSG_MODE_STATE, &st, sizeof(st));
					PX4_INFO("mode '%s' %s", cl.name, now_active ? "ACTIVATED" : "deactivated");
				}
			}
		}

		// 3b) keep the setpoint type pinned while a mode is active (1 Hz refresh;
		// idempotent, guards against any reset path)
		for (int i = 0; i < MAX_CLIENTS; i++) {
			Client &cl = _clients[i];

			if (cl.in_use && cl.px4_ack && cl.active
			    && hrt_elapsed_time(&cl.sp_config_sent) > 1_s) {
				publish_setpoint_config(cl);
			}
		}

		// 4) vehicle-state broadcast (~50 Hz) while anyone is registered
		bool any_client = false;

		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (_clients[i].in_use) { any_client = true; break; }
		}

		if (any_client) {
			vehicle_local_position_s lp{};
			vehicle_attitude_s att{};
			manual_control_setpoint_s man{};
			_lpos_sub.copy(&lp);
			_att_sub.copy(&att);
			const bool man_ok = _manual_sub.copy(&man) && man.valid;

			mode_vehicle_state_t vst{};
			vst.timestamp_us = hrt_absolute_time();
			vst.position[0] = lp.x; vst.position[1] = lp.y; vst.position[2] = lp.z;
			vst.velocity[0] = lp.vx; vst.velocity[1] = lp.vy; vst.velocity[2] = lp.vz;
			vst.q[0] = att.q[0]; vst.q[1] = att.q[1]; vst.q[2] = att.q[2]; vst.q[3] = att.q[3];
			vst.heading = lp.heading;

			if (man_ok) {
				vst.stick_roll = man.roll;
				vst.stick_pitch = man.pitch;
				vst.stick_yaw = man.yaw;
				vst.stick_throttle = man.throttle;
				vst.aux[0] = man.aux1; vst.aux[1] = man.aux2;
				vst.aux[2] = man.aux3; vst.aux[3] = man.aux4;
				vst.aux[4] = man.aux5; vst.aux[5] = man.aux6;
			}

			vst.manual_valid = man_ok;
			vst.armed = _armed;
			vst.nav_state = _nav_state;
			pipe_send(MODE_MSG_VEHICLE_STATE, &vst, sizeof(vst));
		}

		pthread_mutex_unlock(&_mutex);
	}

	// ---- MPA thread ----
	static void control_callback(int ch, char *data, int bytes, void *context)
	{
		FlightModeBridge *self = static_cast<FlightModeBridge *>(context);

		if (self == nullptr || data == nullptr || ch != self->_pipe_ch) { return; }

		while (bytes >= (int)sizeof(mode_msg_header_t)) {
			mode_msg_header_t h;
			memcpy(&h, data, sizeof(h));

			if (h.magic != VOXL_PX4_MODES_MAGIC || h.version != VOXL_PX4_MODES_PROTOCOL_VERSION) { return; }

			const int total = sizeof(mode_msg_header_t) + h.payload_bytes;

			if (bytes < total) { return; }

			self->handle_msg(h.type, data + sizeof(mode_msg_header_t), h.payload_bytes);
			data += total;
			bytes -= total;
		}
	}

	void handle_msg(uint8_t type, const char *pl, uint16_t bytes)
	{
		pthread_mutex_lock(&_mutex);

		switch (type) {
		case MODE_MSG_REGISTER_REQ: {
				if (bytes < sizeof(mode_register_req_t)) { break; }

				mode_register_req_t r;
				memcpy(&r, pl, sizeof(r));

				Client *c = find_client(r.request_id);

				if (c != nullptr && c->px4_ack) {
					// client re-sent registration (missed our reply): answer from state
					mode_register_reply_t out{};
					out.request_id = c->request_id;
					out.success = 1;
					out.mode_id = c->mode_id;
					out.arming_check_id = c->arming_check_id;
					pipe_send(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
					break;
				}

				if (c != nullptr && !c->px4_ack) {
					// registration already forwarded, commander reply still in
					// flight (slow at boot). Forwarding a duplicate would make
					// commander register the mode TWICE and park the second
					// copy in a fresh slot ("already registered" path). Swallow
					// the retry; re-forward only if the request seems lost.
					if (hrt_elapsed_time(&c->reg_forwarded) < 5_s) {
						break;
					}
				}

				if (r.setpoint_type != MODE_SETPOINT_TRAJECTORY && r.setpoint_type != MODE_SETPOINT_ATTITUDE) {
					PX4_WARN("mode '%s': unsupported setpoint type %d; rejecting", r.name, r.setpoint_type);
					mode_register_reply_t out{};
					out.request_id = r.request_id;
					out.success = 0;
					out.mode_id = -1;
					out.arming_check_id = -1;
					pipe_send(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
					break;
				}

				if (c == nullptr) {
					c = alloc_client();
				}

				if (c == nullptr) {
					PX4_WARN("registry full; rejecting '%s'", r.name);
					mode_register_reply_t out{};
					out.request_id = r.request_id;
					out.success = 0;
					out.mode_id = -1;
					out.arming_check_id = -1;
					pipe_send(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
					break;
				}

				c->in_use = true;
				c->request_id = r.request_id;
				strncpy(c->name, r.name, sizeof(c->name) - 1);
				c->setpoint_type = r.setpoint_type;
				c->last_seen = hrt_absolute_time();
				c->reg_forwarded = c->last_seen;

				register_ext_component_request_s req{};
				req.timestamp = hrt_absolute_time();
				req.request_id = r.request_id;
				strncpy(req.name, r.name, sizeof(req.name) - 1);
				req.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
				req.register_arming_check = r.register_arming_check;
				req.register_mode = r.register_mode;
				_reg_req_pub.publish(req);
				PX4_INFO("forwarding registration for mode '%s' (setpoint type %d)", r.name, r.setpoint_type);
				break;
			}

		case MODE_MSG_HEALTH: {
				if (bytes < sizeof(mode_health_t)) { break; }

				mode_health_t hl;
				memcpy(&hl, pl, sizeof(hl));

				Client *c = find_client(hl.request_id);

				if (c != nullptr) {
					c->can_arm = hl.can_arm_and_run;
					c->reqs = hl.mode_requirements;
					c->last_seen = hrt_absolute_time();
				}

				break;
			}

		case MODE_MSG_TRAJ_SETPOINT: {
				if (bytes < sizeof(uint64_t) + sizeof(mavlink_set_position_target_local_ned_t)) { break; }

				uint64_t rid;
				mavlink_set_position_target_local_ned_t t;
				memcpy(&rid, pl, sizeof(rid));
				memcpy(&t, pl + sizeof(rid), sizeof(t));

				Client *c = find_client(rid);

				if (c == nullptr) { break; }

				c->last_seen = hrt_absolute_time();

				if (!c->active) { break; }   // ignore setpoints while not selected

				if (c->setpoint_type != MODE_SETPOINT_TRAJECTORY) { break; }

				if (t.coordinate_frame != MODE_MAV_FRAME_LOCAL_NED) {
					warn_once(_warned_frame, "'%s': only MAV_FRAME_LOCAL_NED supported (got %d)",
						  c->name, t.coordinate_frame);
					break;
				}

				if (t.type_mask & MODE_POS_MASK_FORCE_SET) {
					warn_once(_warned_force, "'%s': FORCE_SET not supported", c->name);
					break;
				}

				// type_mask -> NAN translation, identical to mavlink_receiver.cpp
				const uint16_t m = t.type_mask;
				trajectory_setpoint_s ts{};
				ts.position[0] = (m & MODE_POS_MASK_X_IGNORE) ? NAN : t.x;
				ts.position[1] = (m & MODE_POS_MASK_Y_IGNORE) ? NAN : t.y;
				ts.position[2] = (m & MODE_POS_MASK_Z_IGNORE) ? NAN : t.z;
				ts.velocity[0] = (m & MODE_POS_MASK_VX_IGNORE) ? NAN : t.vx;
				ts.velocity[1] = (m & MODE_POS_MASK_VY_IGNORE) ? NAN : t.vy;
				ts.velocity[2] = (m & MODE_POS_MASK_VZ_IGNORE) ? NAN : t.vz;
				ts.acceleration[0] = (m & MODE_POS_MASK_AX_IGNORE) ? NAN : t.afx;
				ts.acceleration[1] = (m & MODE_POS_MASK_AY_IGNORE) ? NAN : t.afy;
				ts.acceleration[2] = (m & MODE_POS_MASK_AZ_IGNORE) ? NAN : t.afz;
				ts.jerk[0] = NAN; ts.jerk[1] = NAN; ts.jerk[2] = NAN;
				ts.yaw = (m & MODE_POS_MASK_YAW_IGNORE) ? NAN : t.yaw;
				ts.yawspeed = (m & MODE_POS_MASK_YAW_RATE_IGNORE) ? NAN : t.yaw_rate;
				ts.timestamp = hrt_absolute_time();
				_traj_pub.publish(ts);
				break;
			}

		case MODE_MSG_ATT_SETPOINT: {
				if (bytes < sizeof(uint64_t) + sizeof(mavlink_set_attitude_target_t)) { break; }

				uint64_t rid;
				mavlink_set_attitude_target_t a;
				memcpy(&rid, pl, sizeof(rid));
				memcpy(&a, pl + sizeof(rid), sizeof(a));

				Client *c = find_client(rid);

				if (c == nullptr) { break; }

				c->last_seen = hrt_absolute_time();

				if (!c->active) { break; }

				if (c->setpoint_type != MODE_SETPOINT_ATTITUDE) { break; }

				// attitude + thrust required; translation mirrors mavlink_receiver.cpp
				const uint8_t m = a.type_mask;
				const bool attitude = !(m & MODE_ATT_MASK_ATTITUDE_IGNORE);
				const bool thrust_body_set = m & MODE_ATT_MASK_THRUST_BODY_SET;
				const bool thrust = !(m & MODE_ATT_MASK_THROTTLE_IGNORE);

				if (!attitude || !(thrust || thrust_body_set)) {
					warn_once(_warned_att, "'%s': attitude + thrust required in SET_ATTITUDE_TARGET", c->name);
					break;
				}

				vehicle_attitude_setpoint_s as{};
				as.q_d[0] = a.q[0]; as.q_d[1] = a.q[1]; as.q_d[2] = a.q[2]; as.q_d[3] = a.q[3];
				as.yaw_sp_move_rate = (m & MODE_ATT_MASK_BODY_YAW_RATE_IGNORE) ? NAN : a.body_yaw_rate;

				if (thrust_body_set) {
					as.thrust_body[0] = a.thrust_body[0];
					as.thrust_body[1] = a.thrust_body[1];
					as.thrust_body[2] = a.thrust_body[2];

				} else {
					// multicopter: collective thrust maps to -Z in body FRD
					as.thrust_body[0] = 0.f;
					as.thrust_body[1] = 0.f;
					as.thrust_body[2] = -a.thrust;
				}

				as.timestamp = hrt_absolute_time();
				_att_sp_pub.publish(as);
				break;
			}

		case MODE_MSG_UNREGISTER: {
				if (bytes < sizeof(mode_unregister_t)) { break; }

				mode_unregister_t u;
				memcpy(&u, pl, sizeof(u));

				Client *c = find_client(u.request_id);

				if (c == nullptr) { break; }

				unregister_ext_component_s unreg{};
				unreg.timestamp = hrt_absolute_time();
				strncpy(unreg.name, c->name, sizeof(unreg.name) - 1);
				unreg.arming_check_id = c->arming_check_id;
				unreg.mode_id = c->mode_id;
				unreg.mode_executor_id = -1;
				_unreg_pub.publish(unreg);
				PX4_INFO("unregistered mode '%s'", c->name);
				*c = Client{};
				break;
			}

		default:
			break;
		}

		pthread_mutex_unlock(&_mutex);
	}

	template<typename... Args>
	void warn_once(bool &flag, const char *fmt, Args... args)
	{
		if (!flag) {
			PX4_WARN(fmt, args...);
			flag = true;
		}
	}

	int pipe_send(uint8_t type, const void *payload, uint16_t bytes)
	{
		char buf[sizeof(mode_msg_header_t) + 128];
		mode_msg_header_t h{VOXL_PX4_MODES_MAGIC, VOXL_PX4_MODES_PROTOCOL_VERSION, type, bytes};
		memcpy(buf, &h, sizeof(h));
		memcpy(buf + sizeof(h), payload, bytes);
		return MPA::PipeWrite(_pipe_ch, buf, sizeof(h) + bytes);
	}

	static constexpr int MAX_CLIENTS = 8;

	Client *find_client(uint64_t rid)
	{
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (_clients[i].in_use && _clients[i].request_id == rid) { return &_clients[i]; }
		}

		return nullptr;
	}

	Client *alloc_client()
	{
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (!_clients[i].in_use) { return &_clients[i]; }
		}

		return nullptr;
	}

	int _pipe_ch{-1};
	Client _clients[MAX_CLIENTS] {};
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	bool _armed{false};
	uint8_t _nav_state{0};
	bool _warned_frame{false};
	bool _warned_force{false};
	bool _warned_att{false};

	uORB::Publication<register_ext_component_request_s> _reg_req_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unreg_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<arming_check_reply_s> _ac_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<setpoint_config_s> _sp_config_pub{ORB_ID(setpoint_config)};
	uORB::Publication<trajectory_setpoint_s> _traj_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	uORB::Subscription _reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _sp_config_reply_sub{ORB_ID(setpoint_config_reply)};
	uORB::Subscription _ac_req_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vstatus_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
};

ModuleBase::Descriptor FlightModeBridge::desc{FlightModeBridge::task_spawn, FlightModeBridge::custom_command, FlightModeBridge::print_usage};

int FlightModeBridge::task_spawn(int argc, char *argv[])
{
	FlightModeBridge *instance = new FlightModeBridge();

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int FlightModeBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlightModeBridge::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_USAGE_NAME("flight_mode_bridge", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int flight_mode_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(FlightModeBridge::desc, argc, argv);
}
