/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * flight_mode_bridge: MPA <-> PX4 external-flight-modes bridge.
 *
 * Serves the "px4_modes" MPA pipe. Protocol: voxl_px4_modes_pipe.h (a
 * synced copy — the original lives in the apps-side example repo, currently
 * voxl-px4-external-modes; edit there and copy here). Translates pipe messages
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
 * derives the control flags for the mode.
 *
 * Design notes:
 *  - Arming-check polls are answered FROM CACHE (client streams health at
 *    ~1 Hz; any client message refreshes aliveness). Once a client is >3 s
 *    silent the bridge stops answering for it entirely, so commander's own
 *    unresponsive-mode reaper can free the seat — answering on a dead
 *    client's behalf is what would keep that mode registered forever.
 *  - Setpoint loss is enforced HERE, not by PX4: SetpointConfig.timeout_ms
 *    is not consumed by commander in this tree, so the bridge runs its own
 *    watchdog over forwarded setpoints (see setpointStalled()).
 *  - MPA control callback runs on the MPA thread: shared state is mutexed;
 *    uORB publish from that thread follows the crsf_bridge precedent.
 ****************************************************************************/

#include <containers/LockGuard.hpp>
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

// Max gap between forwarded setpoints for an active mode. Enforced by this
// module's watchdog; also published in SetpointConfig.timeout_ms so the value
// is already correct if PX4 ever consumes that field (it does not today).
static constexpr uint16_t SETPOINT_TIMEOUT_MS = 500;

// Minimum spacing between repeats of the same diagnostic warning.
static constexpr uint64_t WARN_INTERVAL_US = 2_s;

// Every message the bridge sends must fit the protocol's declared maximum, so
// that adding one later cannot quietly overrun the send buffer.
static_assert(sizeof(mode_register_reply_t) <= VOXL_PX4_MODES_MAX_PAYLOAD, "register reply exceeds max payload");
static_assert(sizeof(mode_state_t) <= VOXL_PX4_MODES_MAX_PAYLOAD, "mode state exceeds max payload");
static_assert(sizeof(mode_vehicle_state_t) <= VOXL_PX4_MODES_MAX_PAYLOAD, "vehicle state exceeds max payload");

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

		if (MPA::PipeServerSetControlCb(_pipe_ch, &FlightModeBridge::controlCallback, this) < 0) {
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

		// Runs from both Run() (on should_exit) and the destructor; the second
		// call must not touch a mutex the first one destroyed.
		if (_cleaned_up) { return; }

		_cleaned_up = true;

		// Hand every seat back explicitly rather than leaving commander to
		// time the whole bridge out. Note PX4 only processes unregistrations
		// while disarmed (ModeManagement::update), so a stop with the vehicle
		// armed still leaves the modes registered until it disarms.
		{
			LockGuard lock{_mutex};

			for (int i = 0; i < MAX_CLIENTS; i++) {
				if (_clients[i].in_use) {
					PX4_INFO("shutdown: unregistering mode '%s'", _clients[i].name);
					unregisterClient(_clients[i]);
				}
			}
		}

		// Close the pipe before destroying the mutex: no further control
		// callbacks can arrive once the server is gone, so nothing can be
		// waiting on it.
		if (_pipe_ch >= 0) {
			MPA::PipeServerClose(_pipe_ch);
			_pipe_ch = -1;
		}

		pthread_mutex_destroy(&_mutex);
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
		uint32_t reqs{0};           // requirements the client declares
		uint32_t derived_reqs{0};   // requirements PX4 derived from the setpoint type
		hrt_abstime last_seen{0};       // any message from this client
		hrt_abstime last_setpoint{0};   // setpoint messages only (watchdog)
		bool active{false};
		bool sp_timed_out{false};       // watchdog tripped; logged once per stall
		bool reported_stale{false};     // "gone silent" logged once per outage
		hrt_abstime sp_config_sent{0};
		hrt_abstime reg_forwarded{0};
	};

	static bool allNan(const float v[3])
	{
		return !PX4_ISFINITE(v[0]) && !PX4_ISFINITE(v[1]) && !PX4_ISFINITE(v[2]);
	}

	// ---- MAVLink -> uORB setpoint translation ---------------------------------
	//
	// Pure: decoded MAVLink payload in, PX4 setpoint out, no bridge or client
	// state touched and nothing published. The rules mirror
	// src/modules/mavlink/mavlink_receiver.cpp; when those are factored into
	// shared PX4 helpers (see EXTERNAL_MODE_CODE_SHARING.md) these two
	// functions are the only call sites to replace. Keeping them free of
	// module state also makes them unit-testable without a pipe or a vehicle.
	enum class TranslateResult {
		Ok = 0,
		UnsupportedFrame,   // trajectory: frame other than MAV_FRAME_LOCAL_NED
		ForceUnsupported,   // trajectory: FORCE_SET bit
		NoComponents,       // trajectory: no position, velocity or acceleration
		AttitudeRequired,   // attitude: attitude and thrust are both mandatory
		ThrustAxisUnknown,  // attitude: scalar thrust on a non-multicopter
	};

	static TranslateResult translateTrajSetpoint(const mavlink_set_position_target_local_ned_t &in,
			trajectory_setpoint_s &out)
	{
		if (in.coordinate_frame != MODE_MAV_FRAME_LOCAL_NED) {
			return TranslateResult::UnsupportedFrame;
		}

		if (in.type_mask & MODE_POS_MASK_FORCE_SET) {
			return TranslateResult::ForceUnsupported;
		}

		const uint16_t m = in.type_mask;
		out = trajectory_setpoint_s{};
		out.position[0] = (m & MODE_POS_MASK_X_IGNORE) ? NAN : in.x;
		out.position[1] = (m & MODE_POS_MASK_Y_IGNORE) ? NAN : in.y;
		out.position[2] = (m & MODE_POS_MASK_Z_IGNORE) ? NAN : in.z;
		out.velocity[0] = (m & MODE_POS_MASK_VX_IGNORE) ? NAN : in.vx;
		out.velocity[1] = (m & MODE_POS_MASK_VY_IGNORE) ? NAN : in.vy;
		out.velocity[2] = (m & MODE_POS_MASK_VZ_IGNORE) ? NAN : in.vz;
		out.acceleration[0] = (m & MODE_POS_MASK_AX_IGNORE) ? NAN : in.afx;
		out.acceleration[1] = (m & MODE_POS_MASK_AY_IGNORE) ? NAN : in.afy;
		out.acceleration[2] = (m & MODE_POS_MASK_AZ_IGNORE) ? NAN : in.afz;
		out.jerk[0] = NAN; out.jerk[1] = NAN; out.jerk[2] = NAN;
		out.yaw = (m & MODE_POS_MASK_YAW_IGNORE) ? NAN : in.yaw;
		out.yawspeed = (m & MODE_POS_MASK_YAW_RATE_IGNORE) ? NAN : in.yaw_rate;

		// A setpoint with no position, velocity OR acceleration commands
		// nothing; mavlink_receiver rejects it as "invalid, missing position,
		// velocity or acceleration" after the same NAN fill.
		if (allNan(out.position) && allNan(out.velocity) && allNan(out.acceleration)) {
			return TranslateResult::NoComponents;
		}

		return TranslateResult::Ok;
	}

	static TranslateResult translateAttSetpoint(const mavlink_set_attitude_target_t &in, uint8_t vehicle_type,
			vehicle_attitude_setpoint_s &out)
	{
		const uint8_t m = in.type_mask;
		const bool attitude = !(m & MODE_ATT_MASK_ATTITUDE_IGNORE);
		const bool thrust_body_set = m & MODE_ATT_MASK_THRUST_BODY_SET;
		const bool thrust = !(m & MODE_ATT_MASK_THROTTLE_IGNORE);

		if (!attitude || !(thrust || thrust_body_set)) {
			return TranslateResult::AttitudeRequired;
		}

		// Scalar thrust has no axis of its own: PX4's fill_thrust() picks one
		// from the vehicle type (-Z multicopter, +X fixed-wing and rover,
		// VTOL-state-dependent). Only the multicopter convention is
		// implemented here, so refuse rather than push a quad's thrust axis
		// onto a wing; other airframes must send THRUST_BODY_SET.
		if (!thrust_body_set && vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			return TranslateResult::ThrustAxisUnknown;
		}

		out = vehicle_attitude_setpoint_s{};
		out.q_d[0] = in.q[0]; out.q_d[1] = in.q[1]; out.q_d[2] = in.q[2]; out.q_d[3] = in.q[3];
		out.yaw_sp_move_rate = (m & MODE_ATT_MASK_BODY_YAW_RATE_IGNORE) ? NAN : in.body_yaw_rate;

		if (thrust_body_set) {
			out.thrust_body[0] = in.thrust_body[0];
			out.thrust_body[1] = in.thrust_body[1];
			out.thrust_body[2] = in.thrust_body[2];

		} else {
			out.thrust_body[0] = 0.f;
			out.thrust_body[1] = 0.f;
			out.thrust_body[2] = -in.thrust;
		}

		return TranslateResult::Ok;
	}

	// One warning per failure reason, rate-limited, naming the offending mode.
	void reportTranslateFailure(TranslateResult r, const Client &c, const mavlink_set_position_target_local_ned_t *t)
	{
		switch (r) {
		case TranslateResult::UnsupportedFrame:
			warnThrottled(_warned_frame, "'%s': only MAV_FRAME_LOCAL_NED supported (got %d)",
				      c.name, t != nullptr ? t->coordinate_frame : -1);
			break;

		case TranslateResult::ForceUnsupported:
			warnThrottled(_warned_force, "'%s': FORCE_SET not supported", c.name);
			break;

		case TranslateResult::NoComponents:
			warnThrottled(_warned_traj_empty,
				      "'%s': trajectory setpoint has no position, velocity or acceleration; ignoring", c.name);
			break;

		case TranslateResult::AttitudeRequired:
			warnThrottled(_warned_att, "'%s': attitude + thrust required in SET_ATTITUDE_TARGET", c.name);
			break;

		case TranslateResult::ThrustAxisUnknown:
			warnThrottled(_warned_thrust_type,
				      "'%s': scalar thrust is multicopter-only (vehicle_type %d); use THRUST_BODY_SET",
				      c.name, _vehicle_type);
			break;

		case TranslateResult::Ok:
			break;
		}
	}

	void shortPayload(uint8_t type, uint16_t got, size_t need)
	{
		warnThrottled(_warned_short_payload, "message type %d payload too small: %d bytes, need %d",
			      type, got, static_cast<int>(need));
	}

	void rejectRegistration(uint64_t request_id)
	{
		mode_register_reply_t out{};
		out.request_id = request_id;
		out.success = 0;
		out.mode_id = -1;
		out.arming_check_id = -1;
		pipeSend(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
	}

	// No message of any kind for HEALTH_STALE_US: the client is presumed gone
	// (it streams health at ~1 Hz, so this is three missed messages).
	bool clientStale(const Client &c) const
	{
		return hrt_elapsed_time(&c.last_seen) > HEALTH_STALE_US;
	}

	// Release a client's PX4-side registration. Commander frees the mode slot,
	// so the same mode name gets its remembered seat back (COM_MODEn_HASH) on
	// the next registration instead of being pushed to a different nav_state.
	void unregisterClient(Client &c)
	{
		if (c.px4_ack && (c.mode_id >= 0 || c.arming_check_id >= 0)) {
			unregister_ext_component_s unreg{};
			unreg.timestamp = hrt_absolute_time();
			strncpy(unreg.name, c.name, sizeof(unreg.name) - 1);
			unreg.arming_check_id = c.arming_check_id;
			unreg.mode_id = c.mode_id;
			unreg.mode_executor_id = -1;
			_unreg_pub.publish(unreg);
		}

		c = Client{};
	}

	// Setpoint watchdog: an active mode must keep streaming setpoints. A
	// client whose control loop wedges while its health thread keeps ticking
	// still looks alive to last_seen, so aliveness alone cannot catch it.
	// Reporting can_arm_and_run false takes the mode out of modeCanRun() and
	// drops the vehicle into the failsafe ladder (commander polls at 300 ms,
	// so worst-case stall-to-failsafe is SETPOINT_TIMEOUT_MS + ~300 ms).
	bool setpointStalled(const Client &c) const
	{
		return c.active && c.px4_ack
		       && hrt_elapsed_time(&c.last_setpoint) > (SETPOINT_TIMEOUT_MS * 1000ULL);
	}

	// Declare a mode's setpoint type to commander. Commander resets the
	// stored type back to the default (Trajectory) whenever a mode
	// deactivates (see ModeManagement::getSetpointType), so this must be
	// re-sent on every activation. With a stale default, commander enables
	// the wrong controller set for the mode and PX4's own controllers will
	// fight the mode's setpoints on the shared setpoint topics.
	void publishSetpointConfig(Client &c)
	{
		// source_id is the mode's nav_state. Never publish with an invalid
		// mode_id: the uint8 cast would turn -1 into source 255 and configure
		// setpoints for a mode that does not exist.
		if (c.mode_id < 0) {
			PX4_ERR("mode '%s': refusing setpoint config with invalid mode_id %d", c.name, c.mode_id);
			return;
		}

		setpoint_config_s sc{};
		sc.timestamp = hrt_absolute_time();
		sc.type = c.setpoint_type;
		sc.source_id = static_cast<uint8_t>(c.mode_id);
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

		bool broadcast = false;
		bool armed = false;
		uint8_t nav_state = 0;

		{
			LockGuard lock{_mutex};

			// 1) registration replies from commander -> forward to client,
			//    then declare the mode's setpoint type (SetpointConfig)
			register_ext_component_reply_s rep;

			while (_reply_sub.update(&rep)) {
				Client *c = findClient(rep.request_id);

				if (c == nullptr) { continue; }

				c->px4_ack = rep.success;
				c->mode_id = rep.mode_id;
				c->arming_check_id = rep.arming_check_id;

				mode_register_reply_t out{};
				out.request_id = c->request_id;
				out.success = rep.success;
				out.mode_id = rep.mode_id;
				out.arming_check_id = rep.arming_check_id;
				pipeSend(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
				PX4_INFO("mode '%s' registered: mode_id=%d", c->name, rep.mode_id);

				if (rep.success) {
					publishSetpointConfig(*c);
				}
			}

			// 1b) setpoint-config replies: log the verdict AND keep the derived
			// requirements. PX4 does not apply these itself — SetpointConfigReply.msg
			// states "A mode will use these and apply them to the arming check
			// reply (PX4 does not do that itself)" — so a setpoint type's inherent
			// needs (e.g. a trajectory mode needs a local position estimate) only
			// reach commander if the bridge ORs them into every ArmingCheckReply.
			// Without this, requirements are whatever the client declares, and a
			// client that declares nothing could be armed into a mode whose
			// estimator inputs are missing.
			setpoint_config_reply_s scr;

			while (_sp_config_reply_sub.update(&scr)) {
				for (int i = 0; i < MAX_CLIENTS; i++) {
					if (_clients[i].in_use && _clients[i].mode_id == static_cast<int8_t>(scr.source_id)) {
						if (scr.result == setpoint_config_reply_s::RESULT_SUCCESS) {
							uint32_t derived = 0;

							if (scr.mode_req_angular_velocity) { derived |= MODE_REQ_ANGULAR_VELOCITY; }

							if (scr.mode_req_attitude) { derived |= MODE_REQ_ATTITUDE; }

							if (scr.mode_req_local_alt) { derived |= MODE_REQ_LOCAL_ALT; }

							if (scr.mode_req_local_position) { derived |= MODE_REQ_LOCAL_POSITION; }

							_clients[i].derived_reqs = derived;
							PX4_INFO("mode '%s' setpoint type %d accepted (derived reqs 0x%x)",
								 _clients[i].name, scr.type, static_cast<unsigned>(derived));

						} else {
							_clients[i].derived_reqs = 0;
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

					// A client we believe is dead must be answered for by NOBODY:
					// commander flags a silent registration unresponsive after
					// ~900 ms and, while disarmed, reaps the mode itself
					// (ModeManagement::update, "Check for unresponsive modes",
					// which already skips the currently-selected mode). Replying
					// on a dead client's behalf — even with can_arm false — is
					// what keeps that reaper from ever running and lets a crashed
					// mode sit on its seat.
					if (clientStale(cl)) {
						if (!cl.reported_stale) {
							cl.reported_stale = true;
							PX4_WARN("mode '%s': silent for %llu ms; no longer answering arming checks",
								 cl.name, static_cast<unsigned long long>(HEALTH_STALE_US / 1000));
						}

						continue;
					}

					cl.reported_stale = false;

					const bool stalled = setpointStalled(cl);

					if (stalled != cl.sp_timed_out) {
						cl.sp_timed_out = stalled;

						if (stalled) {
							PX4_ERR("mode '%s': no setpoint for %d ms while active; reporting unhealthy",
								cl.name, SETPOINT_TIMEOUT_MS);

						} else {
							PX4_INFO("mode '%s': setpoint stream recovered", cl.name);
						}
					}

					arming_check_reply_s ac{};
					ac.timestamp = hrt_absolute_time();
					ac.request_id = ac_req.request_id;
					ac.registration_id = static_cast<uint8_t>(cl.arming_check_id);
					ac.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
					// client-declared requirements plus the ones PX4 derived from
					// the setpoint type (see 1b); global position and manual
					// control are client-only, SetpointConfigReply has no such
					// fields to derive them from
					const uint32_t reqs = cl.reqs | cl.derived_reqs;

					ac.can_arm_and_run = cl.can_arm && !stalled;
					ac.mode_req_angular_velocity = reqs & MODE_REQ_ANGULAR_VELOCITY;
					ac.mode_req_attitude = reqs & MODE_REQ_ATTITUDE;
					ac.mode_req_local_alt = reqs & MODE_REQ_LOCAL_ALT;
					ac.mode_req_local_position = reqs & MODE_REQ_LOCAL_POSITION;
					ac.mode_req_global_position = reqs & MODE_REQ_GLOBAL_POSITION;
					ac.mode_req_manual_control = reqs & MODE_REQ_MANUAL_CONTROL;
					_ac_reply_pub.publish(ac);
				}
			}

			// 3) activation watching: nav_state enters/leaves a client's slot
			vehicle_status_s vs;

			if (_vstatus_sub.update(&vs)) {
				_armed = (vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_nav_state = vs.nav_state;
				// tracks the VTOL transition state, not just the airframe
				_vehicle_type = vs.vehicle_type;

				for (int i = 0; i < MAX_CLIENTS; i++) {
					Client &cl = _clients[i];

					if (!cl.in_use || !cl.px4_ack) { continue; }

					const bool now_active = (vs.nav_state == static_cast<uint8_t>(cl.mode_id));

					if (now_active != cl.active) {
						cl.active = now_active;

						if (now_active) {
							// commander wiped the type on last deactivation; re-declare
							publishSetpointConfig(cl);
							// arm the watchdog from here: the client gets one full
							// timeout to produce its first setpoint after activation
							cl.last_setpoint = hrt_absolute_time();
							cl.sp_timed_out = false;
						}

						vehicle_local_position_s lp{};
						_lpos_sub.copy(&lp);

						mode_state_t st{};
						st.request_id = cl.request_id;
						st.active = now_active;
						st.armed = _armed;
						st.position[0] = lp.x; st.position[1] = lp.y; st.position[2] = lp.z;
						st.yaw = lp.heading;
						pipeSend(MODE_MSG_MODE_STATE, &st, sizeof(st));
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
					publishSetpointConfig(cl);
				}
			}

			// 4) decide whether anyone needs the broadcast, and snapshot the
			// two fields it takes from shared state. Everything else in the
			// packet comes from uORB, so the packet itself is built and
			// written AFTER the lock is dropped: this is the only write in
			// the module that runs at 50 Hz, and holding the registry lock
			// across a pipe write of unbounded duration would stall setpoint
			// handling on the MPA thread.
			for (int i = 0; i < MAX_CLIENTS; i++) {
				if (_clients[i].in_use) { broadcast = true; break; }
			}

			armed = _armed;
			nav_state = _nav_state;
		}   // registry lock released here

		if (broadcast) {
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
			vst.armed = armed;
			vst.nav_state = nav_state;
			pipeSend(MODE_MSG_VEHICLE_STATE, &vst, sizeof(vst));
		}
	}

	// ---- MPA thread ----
	static void controlCallback(int ch, char *data, int bytes, void *context)
	{
		FlightModeBridge *self = static_cast<FlightModeBridge *>(context);

		if (self == nullptr || data == nullptr || ch != self->_pipe_ch) { return; }

		while (bytes >= static_cast<int>(sizeof(mode_msg_header_t))) {
			mode_msg_header_t h;
			memcpy(&h, data, sizeof(h));

			// A bad header means the stream is not ours or not aligned; there
			// is no resync point, so drop the rest of this buffer. Saying so
			// is the difference between "my client does nothing" and "my
			// client speaks the wrong protocol version".
			if (h.magic != VOXL_PX4_MODES_MAGIC || h.version != VOXL_PX4_MODES_PROTOCOL_VERSION) {
				self->warnThrottled(self->_warned_bad_header,
						    "bad message header (magic 0x%x version %d; expected 0x%x v%d); dropping %d bytes",
						    static_cast<unsigned>(h.magic), h.version, static_cast<unsigned>(VOXL_PX4_MODES_MAGIC),
						    VOXL_PX4_MODES_PROTOCOL_VERSION, bytes);
				return;
			}

			const int total = sizeof(mode_msg_header_t) + h.payload_bytes;

			if (bytes < total) {
				self->warnThrottled(self->_warned_short_payload,
						    "truncated message type %d: need %d bytes, have %d", h.type, total, bytes);
				return;
			}

			self->handleMsg(h.type, data + sizeof(mode_msg_header_t), h.payload_bytes);
			data += total;
			bytes -= total;
		}
	}

	void handleMsg(uint8_t type, const char *pl, uint16_t bytes)
	{
		// RAII: the switch below is all `break`s today, but a `return` slipped
		// in later would leak the lock and wedge both threads permanently.
		LockGuard lock{_mutex};

		switch (type) {
		case MODE_MSG_REGISTER_REQ: {
				if (bytes < sizeof(mode_register_req_t)) {
					shortPayload(MODE_MSG_REGISTER_REQ, bytes, sizeof(mode_register_req_t));
					break;
				}

				mode_register_req_t r;
				memcpy(&r, pl, sizeof(r));
				// wire data: force termination before the name is compared,
				// copied or printed
				r.name[sizeof(r.name) - 1] = '\0';

				// An empty name is not usable: commander seats a mode by the
				// FNV-1a hash of its name and the GCS lists it by name, so a
				// blank one gets a degenerate seat and is unidentifiable.
				if (r.name[0] == '\0') {
					PX4_ERR("registration with empty mode name; rejecting");
					rejectRegistration(r.request_id);
					break;
				}

				// This bridge's contract is exactly one mode per client, with
				// arming checks. Without register_mode commander replies
				// success but mode_id = -1, which would then be published as
				// setpoint source_id 255.
				if (r.register_mode != 1 || r.register_arming_check != 1) {
					PX4_ERR("mode '%s': register_mode=%d register_arming_check=%d; both must be 1",
						r.name, r.register_mode, r.register_arming_check);
					rejectRegistration(r.request_id);
					break;
				}

				Client *c = findClient(r.request_id);

				if (c != nullptr && c->px4_ack) {
					// client re-sent registration (missed our reply): answer from state
					mode_register_reply_t out{};
					out.request_id = c->request_id;
					out.success = 1;
					out.mode_id = c->mode_id;
					out.arming_check_id = c->arming_check_id;
					pipeSend(MODE_MSG_REGISTER_REPLY, &out, sizeof(out));
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
					rejectRegistration(r.request_id);
					break;
				}

				// New request_id, but this name is already on file: a client
				// process restarted (its request_id is random per process).
				// Its old seat is still held, so commander would hit the
				// "already registered" path and hand the mode a DIFFERENT
				// nav_state — which silently moves it to another RC switch
				// position. Free the seat and let the client's 1 Hz retry
				// take it back through the normal COM_MODEn_HASH lookup.
				if (c == nullptr) {
					Client *dup = findClientByName(r.name);

					if (dup != nullptr) {
						if (clientStale(*dup)) {
							PX4_INFO("mode '%s': re-registering; releasing stale seat (mode_id=%d)",
								 dup->name, dup->mode_id);
							unregisterClient(*dup);
							// deliberately not forwarding now: commander frees
							// the slot first, the retry lands in it
							break;
						}

						// Incumbent still looks alive — but usually it is not.
						// A client that crashed and respawned inside the
						// staleness window arrives here, and nothing but time
						// distinguishes it from two live processes sharing a
						// name. So do NOT refuse: stay quiet and let the
						// client's 1 Hz retry come back once the incumbent has
						// gone silent and the branch above can reclaim the
						// seat. A genuine duplicate just keeps warning.
						if (hrt_elapsed_time(&_last_dup_warn) > 2_s) {
							_last_dup_warn = hrt_absolute_time();
							PX4_WARN("mode '%s': name held by a client last heard from %llu ms ago; waiting",
								 r.name, static_cast<unsigned long long>(hrt_elapsed_time(&dup->last_seen) / 1000));
						}

						break;
					}

					c = allocClient();
				}

				if (c == nullptr) {
					PX4_WARN("registry full; rejecting '%s'", r.name);
					rejectRegistration(r.request_id);
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
				if (bytes < sizeof(mode_health_t)) { shortPayload(MODE_MSG_HEALTH, bytes, sizeof(mode_health_t)); break; }

				mode_health_t hl;
				memcpy(&hl, pl, sizeof(hl));

				Client *c = findClient(hl.request_id);

				if (c != nullptr) {
					c->can_arm = hl.can_arm_and_run;
					c->reqs = hl.mode_requirements;
					c->last_seen = hrt_absolute_time();
				}

				break;
			}

		case MODE_MSG_TRAJ_SETPOINT: {
				if (bytes < sizeof(uint64_t) + sizeof(mavlink_set_position_target_local_ned_t)) {
					shortPayload(MODE_MSG_TRAJ_SETPOINT, bytes,
						     sizeof(uint64_t) + sizeof(mavlink_set_position_target_local_ned_t));
					break;
				}

				uint64_t rid;
				mavlink_set_position_target_local_ned_t t;
				memcpy(&rid, pl, sizeof(rid));
				memcpy(&t, pl + sizeof(rid), sizeof(t));

				Client *c = findClient(rid);

				if (c == nullptr) { break; }

				c->last_seen = hrt_absolute_time();

				if (!c->active) { break; }   // ignore setpoints while not selected

				if (c->setpoint_type != MODE_SETPOINT_TRAJECTORY) { break; }

				trajectory_setpoint_s ts{};
				const TranslateResult res = translateTrajSetpoint(t, ts);

				if (res != TranslateResult::Ok) {
					reportTranslateFailure(res, *c, &t);
					break;
				}

				ts.timestamp = hrt_absolute_time();
				_traj_pub.publish(ts);
				// watchdog counts forwarded setpoints only: a client streaming
				// rejected frames is not commanding the vehicle
				c->last_setpoint = ts.timestamp;
				break;
			}

		case MODE_MSG_ATT_SETPOINT: {
				if (bytes < sizeof(uint64_t) + sizeof(mavlink_set_attitude_target_t)) {
					shortPayload(MODE_MSG_ATT_SETPOINT, bytes,
						     sizeof(uint64_t) + sizeof(mavlink_set_attitude_target_t));
					break;
				}

				uint64_t rid;
				mavlink_set_attitude_target_t a;
				memcpy(&rid, pl, sizeof(rid));
				memcpy(&a, pl + sizeof(rid), sizeof(a));

				Client *c = findClient(rid);

				if (c == nullptr) { break; }

				c->last_seen = hrt_absolute_time();

				if (!c->active) { break; }

				if (c->setpoint_type != MODE_SETPOINT_ATTITUDE) { break; }

				vehicle_attitude_setpoint_s as{};
				const TranslateResult res = translateAttSetpoint(a, _vehicle_type, as);

				if (res != TranslateResult::Ok) {
					reportTranslateFailure(res, *c, nullptr);
					break;
				}

				as.timestamp = hrt_absolute_time();
				_att_sp_pub.publish(as);
				c->last_setpoint = as.timestamp;
				break;
			}

		case MODE_MSG_UNREGISTER: {
				if (bytes < sizeof(mode_unregister_t)) {
					shortPayload(MODE_MSG_UNREGISTER, bytes, sizeof(mode_unregister_t));
					break;
				}

				mode_unregister_t u;
				memcpy(&u, pl, sizeof(u));

				Client *c = findClient(u.request_id);

				if (c == nullptr) { break; }

				PX4_INFO("unregistered mode '%s'", c->name);
				unregisterClient(*c);
				break;
			}

		default:
			warnThrottled(_warned_unknown_type, "unknown message type %d (%d byte payload); ignoring",
				      type, bytes);
			break;
		}
	}

	// Rate-limited diagnostic. A one-shot flag would hide everything after the
	// first bad frame for the life of the module, which makes a client that
	// starts misbehaving later invisible.
	template<typename... Args>
	void warnThrottled(hrt_abstime &last, const char *fmt, Args... args)
	{
		if (hrt_elapsed_time(&last) > WARN_INTERVAL_US) {
			last = hrt_absolute_time();
			PX4_WARN(fmt, args...);
		}
	}

	int pipeSend(uint8_t type, const void *payload, uint16_t bytes)
	{
		// Belt to the static_asserts above: nothing reaches the buffer copy
		// without being bounded first.
		if (bytes > VOXL_PX4_MODES_MAX_PAYLOAD) {
			PX4_ERR("message type %d payload %d exceeds %d byte protocol maximum; dropped",
				type, bytes, VOXL_PX4_MODES_MAX_PAYLOAD);
			return -1;
		}

		char buf[sizeof(mode_msg_header_t) + VOXL_PX4_MODES_MAX_PAYLOAD];
		mode_msg_header_t h{VOXL_PX4_MODES_MAGIC, VOXL_PX4_MODES_PROTOCOL_VERSION, type, bytes};
		memcpy(buf, &h, sizeof(h));
		memcpy(buf + sizeof(h), payload, bytes);

		const int ret = MPA::PipeWrite(_pipe_ch, buf, sizeof(h) + bytes);

		// Callers cannot do anything useful with a failed write (a registration
		// reply or activation message is simply lost, and the client's retry
		// covers it), but a silent failure leaves no trace at all.
		if (ret < 0) {
			warnThrottled(_warned_pipe_write, "pipe write failed for message type %d (ret %d)", type, ret);
		}

		return ret;
	}

	static constexpr int MAX_CLIENTS = 8;

	Client *findClient(uint64_t rid)
	{
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (_clients[i].in_use && _clients[i].request_id == rid) { return &_clients[i]; }
		}

		return nullptr;
	}

	Client *findClientByName(const char *name)
	{
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (_clients[i].in_use && strncmp(_clients[i].name, name, sizeof(_clients[i].name)) == 0) {
				return &_clients[i];
			}
		}

		return nullptr;
	}

	Client *allocClient()
	{
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (!_clients[i].in_use) { return &_clients[i]; }
		}

		// Registry full. Stale entries are kept on purpose — they carry the
		// mode/arming ids needed to release the PX4 seat by name — but a dead
		// client must not permanently deny a slot to a live one, so recycle
		// the entry that has been silent longest.
		Client *oldest = nullptr;

		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (clientStale(_clients[i]) && (oldest == nullptr || _clients[i].last_seen < oldest->last_seen)) {
				oldest = &_clients[i];
			}
		}

		if (oldest != nullptr) {
			PX4_WARN("registry full; recycling stale entry for '%s'", oldest->name);
			unregisterClient(*oldest);
			return oldest;
		}

		return nullptr;
	}

	int _pipe_ch{-1};
	Client _clients[MAX_CLIENTS] {};
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	bool _cleaned_up{false};   // cleanup() is reachable twice; see cleanup()
	bool _armed{false};
	uint8_t _nav_state{0};
	uint8_t _vehicle_type{vehicle_status_s::VEHICLE_TYPE_ROTARY_WING};
	hrt_abstime _last_dup_warn{0};
	hrt_abstime _warned_frame{0};
	hrt_abstime _warned_force{0};
	hrt_abstime _warned_att{0};
	hrt_abstime _warned_traj_empty{0};
	hrt_abstime _warned_thrust_type{0};
	hrt_abstime _warned_bad_header{0};
	hrt_abstime _warned_unknown_type{0};
	hrt_abstime _warned_short_payload{0};
	hrt_abstime _warned_pipe_write{0};

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

ModuleBase::Descriptor FlightModeBridge::desc{
	FlightModeBridge::task_spawn,
	FlightModeBridge::custom_command,
	FlightModeBridge::print_usage
};

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
	if (reason != nullptr) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flight mode bridge - serves the px4_modes MPA pipe so apps-processor services can
register named PX4 external flight modes (EXTERNAL1-8) and fly them by streaming
standard MAVLink offboard setpoints (trajectory or attitude).

Each client registers a mode by name, streams health at ~1 Hz, and streams
setpoints while its mode is active. The bridge answers commander's arming checks
on the client's behalf, declares the mode's setpoint type, broadcasts vehicle
state at ~50 Hz, and enforces a setpoint-loss timeout on active modes.

Up to 8 concurrent clients are supported, one mode each.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flight_mode_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int flight_mode_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(FlightModeBridge::desc, argc, argv);
}
