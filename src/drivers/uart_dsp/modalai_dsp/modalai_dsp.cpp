/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include <pthread.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#ifdef __PX4_QURT
#include <drivers/device/qurt/uart.h>
#endif

#include <commander/px4_custom_mode.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <v2.0/mavlink_types.h>
#include <v2.0/standard/mavlink.h>
#include <v2.0/standard/standard.h>
#include <v2.0/protocol.h>
#include <v2.0/mavlink_helpers.h>
#include <v2.0/minimal/mavlink_msg_heartbeat.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <unistd.h>

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"
#define ASYNC_UART_READ_WAIT_US 2000

#ifdef __PX4_QURT
#define MODALAI_ESC_DEFAULT_PORT 	"2"
#else
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"
#endif

#define RC_INPUT_RSSI_MAX	100


extern "C" { __EXPORT int modalai_dsp_main(int argc, char *argv[]); }

namespace modalai_dsp
{

using matrix::wrap_2pi;

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;
bool vio = false;

uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
uORB::Publication<sensor_gps_s>				_gps_pub{ORB_ID(sensor_gps)};
uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};
uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
uORB::PublicationMulti<input_rc_s>			_rc_pub{ORB_ID(input_rc)};
uORB::PublicationMulti<manual_control_setpoint_s>	_manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};

// hil_sensor and hil_state_quaternion
enum SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};

PX4Accelerometer *_px4_accel{nullptr};
PX4Barometer *_px4_baro{nullptr};
PX4Gyroscope *_px4_gyro{nullptr};
PX4Magnetometer *_px4_mag{nullptr};

hrt_abstime _last_heartbeat_check{0};

hrt_abstime _heartbeat_type_antenna_tracker{0};
hrt_abstime _heartbeat_type_gcs{0};
hrt_abstime _heartbeat_type_onboard_controller{0};
hrt_abstime _heartbeat_type_gimbal{0};
hrt_abstime _heartbeat_type_adsb{0};
hrt_abstime _heartbeat_type_camera{0};

hrt_abstime _heartbeat_component_telemetry_radio{0};
hrt_abstime _heartbeat_component_log{0};
hrt_abstime _heartbeat_component_osd{0};
hrt_abstime _heartbeat_component_obstacle_avoidance{0};
hrt_abstime _heartbeat_component_visual_inertial_odometry{0};
hrt_abstime _heartbeat_component_pairing_manager{0};
hrt_abstime _heartbeat_component_udp_bridge{0};
hrt_abstime _heartbeat_component_uart_bridge{0};

bool got_first_sensor_msg = false;
float x_accel = 0;
float y_accel = 0;
float z_accel = 0;
float x_gyro = 0;
float y_gyro = 0;
float z_gyro = 0;
uint64_t gyro_accel_time = 0;

static constexpr unsigned int	MOM_SWITCH_COUNT{8};
uint8_t	_mom_switch_pos[MOM_SWITCH_COUNT] {};
uint16_t _mom_switch_state{0};


vehicle_status_s _vehicle_status{};
vehicle_control_mode_s _control_mode{};
actuator_outputs_s _actuator_outputs{};
actuator_controls_s _actuator_controls{};

int openPort(const char *dev, speed_t speed);
int closePort();

int readResponse(void *buf, size_t len);
int writeResponse(void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
int info();
bool isOpen() { return _uart_fd >= 0; };

void usage();
void task_main(int argc, char *argv[]);

void *send_actuator(void *);
void send_actuator_data();

void handle_message_hil_sensor_dsp(mavlink_message_t *msg);
void handle_message_hil_gps_dsp(mavlink_message_t *msg);
void handle_message_heartbeat_dsp(mavlink_message_t *msg);
void handle_message_odometry_dsp(mavlink_message_t *msg);
void handle_message_vision_position_estimate_dsp(mavlink_message_t *msg);
void handle_message_rc_channels_override_dsp(mavlink_message_t *msg);
void handle_message_manual_control_dsp(mavlink_message_t *msg);

int decode_switch_pos_n_dsp(uint16_t buttons, unsigned sw);
void CheckHeartbeats(const hrt_abstime &t, bool force);
void handle_message_dsp(mavlink_message_t *msg);
void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg);

void
handle_message_dsp(mavlink_message_t *msg)
{
	//PX4_INFO("msg ID: %d", msg->msgid);
	// PX4_ERR("msg ID: %d", msg->msgid);
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor_dsp(msg);
		//PX4_INFO("MAVLINK HIL SENSOR");
		break;
	case MAVLINK_MSG_ID_HIL_GPS:
		if(!vio){
			handle_message_hil_gps_dsp(msg);
			//PX4_INFO("MAVLINK HIL GPS");
			break;
		} else {
			break;
		}
	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		if(vio){
			handle_message_vision_position_estimate_dsp(msg);
			break;
		}
	case MAVLINK_MSG_ID_ODOMETRY:
		if(vio){
			handle_message_odometry_dsp(msg);
			break;
		}
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override_dsp(msg);
		break;
	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control_dsp(msg);
		break;
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		// handle_message_heartbeat_dsp(msg);
		//PX4_INFO("MAVLINK HEART");

		// mavlink_heartbeat_t hb = {};
		// mavlink_message_t message = {};
		// hb.autopilot = 12;
		// hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
		// mavlink_msg_heartbeat_encode(1, 1, &message, &hb);
		//
		// uint8_t  newBuf[MAVLINK_MAX_PACKET_LEN];
		// uint16_t newBufLen = 0;
		// newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
		// int writeRetval = writeResponse(&newBuf, newBufLen);
		// PX4_INFO("Succesful write of heartbeat back to jMAVSim: %d", writeRetval);
		break;
	}
	case MAVLINK_MSG_ID_SYSTEM_TIME:
		// PX4_INFO("MAVLINK SYSTEM TIME");
		break;
	default:
		PX4_ERR("Unknown msg ID: %d", msg->msgid);
		break;
	}
}

void *send_actuator(void *){
	send_actuator_data();
	return nullptr;
}

void send_actuator_data(){

	//int _act_sub = orb_subscribe(ORB_ID(actuator_outputs));
	int _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), 0);
	PX4_ERR("Got %d from orb_subscribe", _actuator_outputs_sub);
	int _vehicle_control_mode_sub_ = orb_subscribe(ORB_ID(vehicle_control_mode));
	PX4_ERR("Got %d from orb_subscribe", _vehicle_control_mode_sub_);

	while (true){

		uint64_t timestamp = hrt_absolute_time();


		bool controls_updated = false;
		(void) orb_check(_vehicle_control_mode_sub_, &controls_updated);

		if(controls_updated){
			orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub_, &_control_mode);
		}

		bool actuator_updated = false;
		(void) orb_check(_actuator_outputs_sub, &actuator_updated);

		//PX4_ERR("Value of actuator_updated: %d", actuator_updated);

		if(actuator_updated){
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
			// PX4_INFO("Value of updated actuator: %d", actuator_updated);
			px4_lockstep_wait_for_components();

			if (_actuator_outputs.timestamp > 0) {
				mavlink_hil_actuator_controls_t hil_act_control;
				actuator_controls_from_outputs_dsp(&hil_act_control);

				mavlink_message_t message{};
				mavlink_msg_hil_actuator_controls_encode(1, 1, &message, &hil_act_control);

				uint8_t  newBuf[512];
				uint16_t newBufLen = 0;
				newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
					int writeRetval = writeResponse(&newBuf, newBufLen);
					PX4_DEBUG("Succesful write of actuator back to jMAVSim: %d at %llu", writeRetval, hrt_absolute_time());
			}
		}

		uint64_t elapsed_time = hrt_absolute_time() - timestamp;
		// if (elapsed_time < 10000) usleep(10000 - elapsed_time);
		if (elapsed_time < 5000) usleep(5000 - elapsed_time);
	}
}

void task_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;
	while ((ch = px4_getopt(argc, argv, "v", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':
			vio = true;
			break;
		default:
			break;
		}
	}

	// int openRetval = openPort(MODALAI_ESC_DEFAULT_PORT, 250000);
	int openRetval = openPort(MODALAI_ESC_DEFAULT_PORT, 921600);
	int open = isOpen();
	if(open){
		PX4_ERR("Port is open: %d", openRetval);
	}

	uint64_t last_heartbeat_timestamp = hrt_absolute_time();
	uint64_t last_imu_update_timestamp = last_heartbeat_timestamp;

	_px4_accel = new PX4Accelerometer(1310988);
	_px4_gyro = new PX4Gyroscope(1310988);

	// Create a thread for sending data to the simulator.
	pthread_t sender_thread;
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));
	pthread_create(&sender_thread, &sender_thread_attr, send_actuator, nullptr);
	pthread_attr_destroy(&sender_thread_attr);

	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	PX4_ERR("Got %d from orb_subscribe", _vehicle_status_sub);
	int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls));
	PX4_ERR("Got %d from orb_subscribe", _actuator_controls_sub);

	bool vehicle_updated = false;
	(void) orb_check(_vehicle_status_sub, &vehicle_updated);

	while (!_task_should_exit){

		uint8_t rx_buf[1024];
		//rx_buf[511] = '\0';

		uint64_t timestamp = hrt_absolute_time();

		// Send out sensor messages every 10ms
		if (got_first_sensor_msg) {
			uint64_t delta_time = timestamp - last_imu_update_timestamp;
			if (delta_time > 15000) {
				PX4_ERR("Sending updates at %llu, delta %llu", timestamp, delta_time);
			}
			uint64_t _px4_gyro_accle_timestamp = hrt_absolute_time();
			_px4_gyro->update(_px4_gyro_accle_timestamp, x_gyro, y_gyro, z_gyro);
			_px4_accel->update(_px4_gyro_accle_timestamp, x_accel, y_accel, z_accel);
			last_imu_update_timestamp = timestamp;
		}

		// Check for incoming messages from the simulator
		int readRetval = readResponse(&rx_buf[0], sizeof(rx_buf));
		if (readRetval) {
		 	// PX4_INFO("Value of rx_buff: %s", rx_buf);
		    	// PX4_INFO("Got %d bytes", readRetval);
			//Take readRetval and convert it into mavlink msg
			mavlink_message_t msg;
			mavlink_status_t _status{};
			for (int i = 0; i <= readRetval; i++){
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &_status)) {
					handle_message_dsp(&msg);
				}
			}
		}
		if ((timestamp - last_heartbeat_timestamp) > 1000000) {
			mavlink_heartbeat_t hb = {};
			mavlink_message_t hb_message = {};
			hb.autopilot = 12;
			hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
			mavlink_msg_heartbeat_encode(1, 1, &hb_message, &hb);

			uint8_t  hb_newBuf[MAVLINK_MAX_PACKET_LEN];
			uint16_t hb_newBufLen = 0;
			hb_newBufLen = mavlink_msg_to_send_buffer(hb_newBuf, &hb_message);
			(void) writeResponse(&hb_newBuf, hb_newBufLen);
			last_heartbeat_timestamp = timestamp;
		} else if (vehicle_updated){
			// PX4_INFO("Value of updated vehicle status: %d", vehicle_updated);
			orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		}

		uint64_t elapsed_time = hrt_absolute_time() - timestamp;
		// if (elapsed_time < 10000) usleep(10000 - elapsed_time);
		if (elapsed_time < 5000) usleep(5000 - elapsed_time);
	}
}

void
handle_message_manual_control_dsp(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	// Check target
	if (man.target != 0) {
		return;
	}

	if (_vehicle_status.rc_input_mode == vehicle_status_s::RC_IN_MODE_GENERATED) {

		input_rc_s rc{};
		rc.timestamp = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp;

		rc.channel_count = 8;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = RC_INPUT_RSSI_MAX;

		rc.values[0] = man.x / 2 + 1500;	// roll
		rc.values[1] = man.y / 2 + 1500;	// pitch
		rc.values[2] = man.r / 2 + 1500;	// yaw
		rc.values[3] = math::constrain(man.z / 0.9f + 800.0f, 1000.0f, 2000.0f);	// throttle

		/* decode all switches which fit into the channel mask */
		unsigned max_switch = (sizeof(man.buttons) * 8);
		unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));

		if (max_switch > (max_channels - 4)) {
			max_switch = (max_channels - 4);
		}

		/* fill all channels */
		for (unsigned i = 0; i < max_switch; i++) {
			rc.values[i + 4] = decode_switch_pos_n_dsp(man.buttons, i);
		}

		_mom_switch_state = man.buttons;

		_rc_pub.publish(rc);

	} else {
		manual_control_setpoint_s manual{};

		manual.timestamp = hrt_absolute_time();
		manual.x = man.x / 1000.0f;
		manual.y = man.y / 1000.0f;
		manual.r = man.r / 1000.0f;
		manual.z = man.z / 1000.0f;
		manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + 1;

		_manual_control_setpoint_pub.publish(manual);
	}
}

int
decode_switch_pos_n_dsp(uint16_t buttons, unsigned sw)
{
	bool on = (buttons & (1 << sw));

	if (sw < MOM_SWITCH_COUNT) {

		bool last_on = (_mom_switch_state & (1 << sw));

		/* first switch is 2-pos, rest is 2 pos */
		unsigned state_count = (sw == 0) ? 3 : 2;

		/* only transition on low state */
		if (!on && (on != last_on)) {

			_mom_switch_pos[sw]++;

			if (_mom_switch_pos[sw] == state_count) {
				_mom_switch_pos[sw] = 0;
			}
		}

		/* state_count - 1 is the number of intervals and 1000 is the range,
		 * with 2 states 0 becomes 0, 1 becomes 1000. With
		 * 3 states 0 becomes 0, 1 becomes 500, 2 becomes 1000,
		 * and so on for more states.
		 */
		return (_mom_switch_pos[sw] * 1000) / (state_count - 1) + 1000;

	} else {
		/* return the current state */
		return on * 1000 + 1000;
	}
}


void
handle_message_rc_channels_override_dsp(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0) {
		return;
	}

	// fill uORB message
	input_rc_s rc{};

	// metadata
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;
	rc.rssi = RC_INPUT_RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;

	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

	// check how many channels are valid
	for (int i = 17; i >= 0; i--) {
		const bool ignore_max = rc.values[i] == UINT16_MAX; // ignore any channel with value UINT16_MAX
		const bool ignore_zero = (i > 7) && (rc.values[i] == 0); // ignore channel 8-18 if value is 0

		if (ignore_max || ignore_zero) {
			// set all ignored values to zero
			rc.values[i] = 0;

		} else {
			// first channel to not ignore -> set count considering zero-based index
			rc.channel_count = i + 1;
			break;
		}
	}

	// publish uORB message
	_rc_pub.publish(rc);
}

void
handle_message_vision_position_estimate_dsp(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t ev;
	mavlink_msg_vision_position_estimate_decode(msg, &ev);

	vehicle_odometry_s visual_odom{};

	uint64_t timestamp = hrt_absolute_time();

	visual_odom.timestamp = timestamp;
	visual_odom.timestamp_sample = timestamp;

	visual_odom.x = ev.x;
	visual_odom.y = ev.y;
	visual_odom.z = ev.z;
	matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
	q.copyTo(visual_odom.q);

	visual_odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	const size_t URT_SIZE = sizeof(visual_odom.pose_covariance) / sizeof(visual_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		visual_odom.pose_covariance[i] = ev.covariance[i];
	}

	visual_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	visual_odom.vx = NAN;
	visual_odom.vy = NAN;
	visual_odom.vz = NAN;
	visual_odom.rollspeed = NAN;
	visual_odom.pitchspeed = NAN;
	visual_odom.yawspeed = NAN;
	visual_odom.velocity_covariance[0] = NAN;

	_visual_odometry_pub.publish(visual_odom);
}

void
handle_message_odometry_dsp(mavlink_message_t *msg)
{
	mavlink_odometry_t odom;
	mavlink_msg_odometry_decode(msg, &odom);

	vehicle_odometry_s odometry{};

	uint64_t timestamp = hrt_absolute_time();

	odometry.timestamp = timestamp;
	odometry.timestamp_sample = timestamp;

	/* The position is in a local FRD frame */
	odometry.x = odom.x;
	odometry.y = odom.y;
	odometry.z = odom.z;

	/**
	 * The quaternion of the ODOMETRY msg represents a rotation from body frame
	 * to a local frame
	 */
	matrix::Quatf q_body_to_local(odom.q);
	q_body_to_local.normalize();
	q_body_to_local.copyTo(odometry.q);

	// pose_covariance
	static constexpr size_t POS_URT_SIZE = sizeof(odometry.pose_covariance) / sizeof(odometry.pose_covariance[0]);
	static_assert(POS_URT_SIZE == (sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	// velocity_covariance
	static constexpr size_t VEL_URT_SIZE = sizeof(odometry.velocity_covariance) / sizeof(odometry.velocity_covariance[0]);
	static_assert(VEL_URT_SIZE == (sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0])),
		      "Odometry Velocity Covariance matrix URT array size mismatch");

	// TODO: create a method to simplify covariance copy
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odometry.pose_covariance[i] = odom.pose_covariance[i];
	}

	/**
	 * PX4 expects the body's linear velocity in the local frame,
	 * the linear velocity is rotated from the odom child_frame to the
	 * local NED frame. The angular velocity needs to be expressed in the
	 * body (fcu_frd) frame.
	 */
	if (odom.child_frame_id == MAV_FRAME_BODY_FRD) {

		odometry.velocity_frame = vehicle_odometry_s::BODY_FRAME_FRD;
		odometry.vx = odom.vx;
		odometry.vy = odom.vy;
		odometry.vz = odom.vz;

		odometry.rollspeed = odom.rollspeed;
		odometry.pitchspeed = odom.pitchspeed;
		odometry.yawspeed = odom.yawspeed;

		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			odometry.velocity_covariance[i] = odom.velocity_covariance[i];
		}

	} else {
		PX4_ERR("Body frame %u not supported. Unable to publish velocity", odom.child_frame_id);
	}

	/**
	 * Supported local frame of reference is MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_FRD
	 * The supported sources of the data/tesimator type are MAV_ESTIMATOR_TYPE_VISION,
	 * MAV_ESTIMATOR_TYPE_VIO and MAV_ESTIMATOR_TYPE_MOCAP
	 *
	 * @note Regarding the local frames of reference, the appropriate EKF_AID_MASK
	 * should be set in order to match a frame aligned (NED) or not aligned (FRD)
	 * with true North
	 */
	if (odom.frame_id == MAV_FRAME_LOCAL_NED || odom.frame_id == MAV_FRAME_LOCAL_FRD) {

		if (odom.frame_id == MAV_FRAME_LOCAL_NED) {
			odometry.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

		} else {
			odometry.local_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
		}

		if (odom.estimator_type == MAV_ESTIMATOR_TYPE_VISION || odom.estimator_type == MAV_ESTIMATOR_TYPE_VIO) {
			_visual_odometry_pub.publish(odometry);

		} else if (odom.estimator_type == MAV_ESTIMATOR_TYPE_MOCAP) {
			_mocap_odometry_pub.publish(odometry);

		} else {
			PX4_ERR("Estimator source %u not supported. Unable to publish pose and velocity", odom.estimator_type);
		}

	} else {
		PX4_ERR("Local frame %u not supported. Unable to publish pose and velocity", odom.frame_id);
	}
}

void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time();

	static constexpr float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

	for (unsigned i = 0; i < 16; i++) {
		if (_actuator_outputs.output[i] > PWM_DEFAULT_MIN / 2) {
			if (i < 4) {
				/* scale PWM out 900..2100 us to 0..1 for rotors */
				msg->controls[i] = (_actuator_outputs.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

			} else {
				/* scale PWM out 900..2100 us to -1..1 for other channels */
				msg->controls[i] = (_actuator_outputs.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
			}
		} else {
			/* send 0 when disarmed and for disabled channels */
			msg->controls[i] = 0.0f;
		}
	}

	msg->mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	if (_control_mode.flag_control_auto_enabled) {
		msg->mode |= MAV_MODE_FLAG_AUTO_ENABLED;
	}

	if (_control_mode.flag_control_manual_enabled) {
		msg->mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}

	if (_control_mode.flag_control_attitude_enabled) {
		msg->mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
	}

	if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		msg->mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	if (_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
		msg->mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		msg->mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
	}

	msg->flags = 0;
}

int openPort(const char *dev, speed_t speed)
{
	if (_uart_fd >= 0) {
		PX4_ERR("Port in use: %s (%i)", dev, errno);
		return -1;
	}

#ifdef __PX4_QURT
	_uart_fd = qurt_uart_open(dev, speed);
	PX4_DEBUG("qurt_uart_opened");
#else
	/* Open UART */
	_uart_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
#endif

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port: %s (%i)", dev, errno);
		return -1;
	}

#ifndef __PX4_QURT
	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(_uart_fd, &_orig_cfg)) < 0) {
		PX4_ERR("Error configuring port: tcgetattr %s: %d", dev, termios_state);
		uart_close();
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart_fd, &_cfg);

	/* Disable output post-processing */
	_cfg.c_oflag &= ~OPOST;

	_cfg.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	_cfg.c_cflag &= ~CSIZE;
	_cfg.c_cflag |= CS8;                 /* 8-bit characters */
	_cfg.c_cflag &= ~PARENB;             /* no parity bit */
	_cfg.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
	_cfg.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	_cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	_cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	if (cfsetispeed(&_cfg, speed) < 0 || cfsetospeed(&_cfg, speed) < 0) {
		PX4_ERR("Error configuring port: %s: %d (cfsetispeed, cfsetospeed)", dev, termios_state);
		uart_close();
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &_cfg)) < 0) {
		PX4_ERR("Error configuring port: %s (tcsetattr)", dev);
		uart_close();
		return -1;
	}
#endif

	return 0;
}

int closePort()
{
#ifndef __PX4_QURT
	if (_uart_fd < 0) {
		PX4_ERR("invalid state for closing");
		return -1;
	}

	if (tcsetattr(_uart_fd, TCSANOW, &_orig_cfg)) {
		PX4_ERR("failed restoring uart to original state");
	}

	if (close(_uart_fd)) {
		PX4_ERR("error closing uart");
	}
#endif

	_uart_fd = -1;

	return 0;
}

int readResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for reading or buffer");
		return -1;
	// } else {
	// 	PX4_INFO("Reading UART");
	}

#ifdef __PX4_QURT
#define ASYNC_UART_READ_WAIT_US 2000
    // The UART read on SLPI is via an asynchronous service so specify a timeout
    // for the return. The driver will poll periodically until the read comes in
    // so this may block for a while. However, it will timeout if no read comes in.
    return qurt_uart_read(_uart_fd, (char*) buf, len, ASYNC_UART_READ_WAIT_US);
#else
	return read(_uart_fd, buf, len);
#endif
}

int writeResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for writing or buffer");
		return -1;
	}

#ifdef __PX4_QURT
    return qurt_uart_write(_uart_fd, (const char*) buf, len);
#else
	return write(_uart_fd, buf, len);
#endif
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("modalai_dsp__main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

int info()
{
	PX4_INFO("running: %s", _is_running ? "yes" : "no");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: modalai_dsp {start|info|stop}");
}

uint64_t first_sensor_msg_timestamp = 0;
uint64_t first_sensor_report_timestamp = 0;
uint64_t last_sensor_report_timestamp = 0;

void
handle_message_hil_sensor_dsp(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t hil_sensor;
	mavlink_msg_hil_sensor_decode(msg, &hil_sensor);

	// temperature only updated with baro
	gyro_accel_time = hrt_absolute_time();

	// temperature only updated with baro
	float temperature = NAN;

	got_first_sensor_msg = true;

	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		temperature = hil_sensor.temperature;
	}

	// gyro
	if ((hil_sensor.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}
		}

		x_gyro = hil_sensor.xgyro;
		y_gyro = hil_sensor.ygyro;
		z_gyro = hil_sensor.zgyro;
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}
		}

		x_accel = hil_sensor.xacc;
		y_accel = hil_sensor.yacc;
		z_accel = hil_sensor.zacc;
	}

	// magnetometer
	if(!vio){
		if ((hil_sensor.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
			if (_px4_mag == nullptr) {
				// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
				_px4_mag = new PX4Magnetometer(197388);
			}

			if (_px4_mag != nullptr) {
				if (PX4_ISFINITE(temperature)) {
					_px4_mag->set_temperature(temperature);
				}

				_px4_mag->update(gyro_accel_time, hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag);
			}
		}
	}

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (_px4_baro == nullptr) {
			// 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
			_px4_baro = new PX4Barometer(6620172);
		}

		if (_px4_baro != nullptr) {
			_px4_baro->set_temperature(hil_sensor.temperature);
			_px4_baro->update(gyro_accel_time, hil_sensor.abs_pressure);
		}
	}

	// differential pressure
	if ((hil_sensor.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {
		differential_pressure_s report{};
		report.timestamp = gyro_accel_time;
		report.temperature = hil_sensor.temperature;
		report.differential_pressure_filtered_pa = hil_sensor.diff_pressure * 100.0f; // convert from millibar to bar;
		report.differential_pressure_raw_pa = hil_sensor.diff_pressure * 100.0f; // convert from millibar to bar;

		_differential_pressure_pub.publish(report);
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = gyro_accel_time;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		_battery_pub.publish(hil_battery_status);
	}
}

uint64_t first_gps_msg_timestamp = 0;
uint64_t first_gps_report_timestamp = 0;

void
handle_message_hil_gps_dsp(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	sensor_gps_s hil_gps{};
	const uint64_t timestamp = hrt_absolute_time();

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	_gps_pub.publish(hil_gps);
}

void
handle_message_heartbeat_dsp(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	const hrt_abstime now = hrt_absolute_time();

	mavlink_heartbeat_t hb;
	mavlink_msg_heartbeat_decode(msg, &hb);

	const bool same_system = 1;

	if (same_system || hb.type == MAV_TYPE_GCS) {

		switch (hb.type) {
		case MAV_TYPE_ANTENNA_TRACKER:
			_heartbeat_type_antenna_tracker = now;
			break;

		case MAV_TYPE_GCS:
			_heartbeat_type_gcs = now;
			break;

		case MAV_TYPE_ONBOARD_CONTROLLER:
			_heartbeat_type_onboard_controller = now;
			break;

		case MAV_TYPE_GIMBAL:
			_heartbeat_type_gimbal = now;
			break;

		case MAV_TYPE_ADSB:
			_heartbeat_type_adsb = now;
			break;

		case MAV_TYPE_CAMERA:
			_heartbeat_type_camera = now;
			break;

		default:
			PX4_INFO("unhandled HEARTBEAT MAV_TYPE: %d from SYSID: %d, COMPID: %d", hb.type, msg->sysid, msg->compid);
		}


		switch (msg->compid) {
		case MAV_COMP_ID_TELEMETRY_RADIO:
			_heartbeat_component_telemetry_radio = now;
			break;

		case MAV_COMP_ID_LOG:
			_heartbeat_component_log = now;
			break;

		case MAV_COMP_ID_OSD:
			_heartbeat_component_osd = now;
			break;

		case MAV_COMP_ID_OBSTACLE_AVOIDANCE:
			_heartbeat_component_obstacle_avoidance = now;
			//_mavlink->telemetry_status().avoidance_system_healthy = (hb.system_status == MAV_STATE_ACTIVE);
			break;

		case MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY:
			_heartbeat_component_visual_inertial_odometry = now;
			break;

		case MAV_COMP_ID_PAIRING_MANAGER:
			_heartbeat_component_pairing_manager = now;
			break;

		case MAV_COMP_ID_UDP_BRIDGE:
			_heartbeat_component_udp_bridge = now;
			break;

		case MAV_COMP_ID_UART_BRIDGE:
			_heartbeat_component_uart_bridge = now;
			break;

		default:
			PX4_INFO("unhandled HEARTBEAT MAV_TYPE: %d from SYSID: %d, COMPID: %d", hb.type, msg->sysid, msg->compid);
		}

		CheckHeartbeats(now, true);
	}

}

void CheckHeartbeats(const hrt_abstime &t, bool force)
{
	// check HEARTBEATs for timeout
	static constexpr uint64_t TIMEOUT = telemetry_status_s::HEARTBEAT_TIMEOUT_US;

	if (t <= TIMEOUT) {
		return;
	}

	//int _telemetry_status_sub = orb_subscribe(ORB_ID(telemetry_status));
		// 	vehicle_control_mode_s control_mode;

		// 	if (_vehicle_control_mode_sub.copy(&control_mode)) {
	// if ((t >= _last_heartbeat_check + (TIMEOUT / 2)) || force) {
	// 	telemetry_status_s tstatus;

	// 	tstatus.heartbeat_type_antenna_tracker         = (t <= TIMEOUT + _heartbeat_type_antenna_tracker);
	// 	tstatus.heartbeat_type_gcs                     = (t <= TIMEOUT + _heartbeat_type_gcs);
	// 	tstatus.heartbeat_type_onboard_controller      = (t <= TIMEOUT + _heartbeat_type_onboard_controller);
	// 	tstatus.heartbeat_type_gimbal                  = (t <= TIMEOUT + _heartbeat_type_gimbal);
	// 	tstatus.heartbeat_type_adsb                    = (t <= TIMEOUT + _heartbeat_type_adsb);
	// 	tstatus.heartbeat_type_camera                  = (t <= TIMEOUT + _heartbeat_type_camera);

	// 	tstatus.heartbeat_component_telemetry_radio    = (t <= TIMEOUT + _heartbeat_component_telemetry_radio);
	// 	tstatus.heartbeat_component_log                = (t <= TIMEOUT + _heartbeat_component_log);
	// 	tstatus.heartbeat_component_osd                = (t <= TIMEOUT + _heartbeat_component_osd);
	// 	tstatus.heartbeat_component_obstacle_avoidance = (t <= TIMEOUT + _heartbeat_component_obstacle_avoidance);
	// 	tstatus.heartbeat_component_vio                = (t <= TIMEOUT + _heartbeat_component_visual_inertial_odometry);
	// 	tstatus.heartbeat_component_pairing_manager    = (t <= TIMEOUT + _heartbeat_component_pairing_manager);
	// 	tstatus.heartbeat_component_udp_bridge         = (t <= TIMEOUT + _heartbeat_component_udp_bridge);
	// 	tstatus.heartbeat_component_uart_bridge        = (t <= TIMEOUT + _heartbeat_component_uart_bridge);

	// 	//_mavlink->telemetry_status_updated();
	// 	_last_heartbeat_check = t;
	// }
}

}
int modalai_dsp_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		modalai_dsp::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return modalai_dsp::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return modalai_dsp::stop();
	}

	else if (!strcmp(verb, "info")) {
		return modalai_dsp::info();
	}

	else {
		modalai_dsp::usage();
		return 1;
	}
}
