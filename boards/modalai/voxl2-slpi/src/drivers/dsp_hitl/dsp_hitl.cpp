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

#include <iostream>
#include <string>
#include <pthread.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <drivers/device/qurt/uart.h>

#include <commander/px4_custom_mode.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <mavlink.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/esc_status.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/drivers/device/Device.hpp> // For DeviceId union

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <unistd.h>

#define ASYNC_UART_READ_WAIT_US 2000
#define RC_INPUT_RSSI_MAX	100

extern "C" { __EXPORT int dsp_hitl_main(int argc, char *argv[]); }

namespace dsp_hitl
{

using matrix::wrap_2pi;

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;
bool vio = false;
bool debug = false;
bool crossfire = false;
bool radio_once = false;
bool rc_once = false;
std::string port = "2";
int baudrate = 921600;
const unsigned mode_flag_custom = 1;
const unsigned mode_flag_armed = 128;

uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
uORB::PublicationMulti<sensor_gps_s>			_sensor_gps_pub{ORB_ID(sensor_gps)};
uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};
uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
uORB::PublicationMulti<input_rc_s>			_rc_pub{ORB_ID(input_rc)};
uORB::PublicationMulti<sensor_baro_s>			_sensor_baro_pub{ORB_ID(sensor_baro)};
uORB::Publication<esc_status_s>			_esc_status_pub{ORB_ID(esc_status)};
uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

int32_t _output_functions[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS] {};

// hil_sensor and hil_state_quaternion
enum SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};

PX4Accelerometer *_px4_accel{nullptr};
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

bool _use_software_mav_throttling{false};
bool _radio_status_available{false};
bool _radio_status_critical{false};
float _radio_status_mult{1.0f};

vehicle_status_s _vehicle_status{};
vehicle_control_mode_s _control_mode{};
actuator_outputs_s _actuator_outputs{};
radio_status_s _rstatus {};
battery_status_s _battery_status{};

sensor_accel_fifo_s accel_fifo{};
sensor_gyro_fifo_s gyro_fifo{};


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
void handle_message_odometry_dsp(mavlink_message_t *msg);
void handle_message_vision_position_estimate_dsp(mavlink_message_t *msg);
void handle_message_rc_channels_override_dsp(mavlink_message_t *msg);
void handle_message_radio_status_dsp(mavlink_message_t *msg);
void handle_message_command_long_dsp(mavlink_message_t *msg);

void CheckHeartbeats(const hrt_abstime &t, bool force);
void handle_message_dsp(mavlink_message_t *msg);
void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg);
void send_esc_telemetry_dsp(mavlink_hil_actuator_controls_t hil_act_control);

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
	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status_dsp(msg);
		break;
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long_dsp(msg);
		break;
	case MAVLINK_MSG_ID_HEARTBEAT:
		//PX4_INFO("Heartbeat msg received");
		break;
	case MAVLINK_MSG_ID_SYSTEM_TIME:
		// PX4_INFO("MAVLINK SYSTEM TIME");
		break;
	default:
		//PX4_ERR("Unknown msg ID: %d", msg->msgid);
		break;
	}
}

void *send_actuator(void *){
	send_actuator_data();
	return nullptr;
}

void send_actuator_data(){

	//int _act_sub = orb_subscribe(ORB_ID(actuator_outputs));
	int _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs_sim), 0);
	//PX4_INFO("Got %d from orb_subscribe", _actuator_outputs_sub);
	int _vehicle_control_mode_sub_ = orb_subscribe(ORB_ID(vehicle_control_mode));
	//PX4_INFO("Got %d from orb_subscribe", _vehicle_control_mode_sub_);
	int previous_timestamp = 0;
	int previous_uorb_timestamp = 0;
	int differential = 0;
	bool first_sent = false;

	while (true){

		//uint64_t timestamp = hrt_absolute_time();

		bool controls_updated = false;
		(void) orb_check(_vehicle_control_mode_sub_, &controls_updated);

		if(controls_updated){
			orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub_, &_control_mode);
		}

		bool actuator_updated = false;
		(void) orb_check(_actuator_outputs_sub, &actuator_updated);

		if(actuator_updated){
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
			px4_lockstep_wait_for_components();
			if (_actuator_outputs.timestamp > 0) {
				mavlink_hil_actuator_controls_t hil_act_control;
				actuator_controls_from_outputs_dsp(&hil_act_control);

				mavlink_message_t message{};
				mavlink_msg_hil_actuator_controls_encode(1, 1, &message, &hil_act_control);
				//differential = _actuator_outputs.timestamp - previous_timestamp;
				//PX4_INFO("Value of internal diff differential: %i", _actuator_outputs.timestamp - previous_timestamp);
				//PX4_INFO("Sending from actuator updated: %i", _actuator_outputs.timestamp - previous_uorb_timestamp);
				previous_timestamp = _actuator_outputs.timestamp;
				previous_uorb_timestamp = _actuator_outputs.timestamp;
				uint8_t  newBuf[512];
				uint16_t newBufLen = 0;
				newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
				int writeRetval = writeResponse(&newBuf, newBufLen);
				PX4_DEBUG("Succesful write of actuator back to jMAVSim: %d at %llu", writeRetval, hrt_absolute_time());
				first_sent = true;
				send_esc_telemetry_dsp(hil_act_control);
			}
		} else if(!actuator_updated && first_sent && differential > 4000){
			mavlink_hil_actuator_controls_t hil_act_control;
			actuator_controls_from_outputs_dsp(&hil_act_control);
			previous_timestamp = hrt_absolute_time();

			mavlink_message_t message{};
			mavlink_msg_hil_actuator_controls_encode(1, 1, &message, &hil_act_control);
			uint8_t  newBuf[512];
			uint16_t newBufLen = 0;
			newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
			int writeRetval = writeResponse(&newBuf, newBufLen);
			//PX4_INFO("Sending from NOT UPDTE AND TIMEOUT: %i", differential);

			PX4_DEBUG("Succesful write of actuator back to jMAVSim: %d at %llu", writeRetval, hrt_absolute_time());
			send_esc_telemetry_dsp(hil_act_control);
		}
		differential = hrt_absolute_time() - previous_timestamp;
		
		//uint64_t elapsed_time = hrt_absolute_time() - timestamp;
		// if (elapsed_time < 10000) usleep(10000 - elapsed_time);
		//if (elapsed_time < 5000) usleep(5000 - elapsed_time);
	}
}

void task_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "vsdcp:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':
			vio = true;
			break;
		case 's':
			_use_software_mav_throttling = true;
			break;
		case 'd':
			debug = true;
			break;
		case 'p':
			port = myoptarg;
			break;
		case 'b':
			baudrate = atoi(myoptarg);
			break;
		case 'c':
			crossfire = true;
			break;
		default:
			break;
		}
	}

	const char* charport = port.c_str();
	int openRetval = openPort(charport, (speed_t) baudrate);
	int open = isOpen();
	if(open){
		PX4_ERR("Port is open: %d", openRetval);
	}

	uint64_t last_heartbeat_timestamp = hrt_absolute_time();
	//uint64_t last_imu_update_timestamp = last_heartbeat_timestamp;

	_px4_accel = new PX4Accelerometer(1310988);
	_px4_gyro = new PX4Gyroscope(1310988);
	_px4_mag = new PX4Magnetometer(197388);

	// Create a thread for sending data to the simulator.
	if(!crossfire){
		pthread_t sender_thread;
		pthread_attr_t sender_thread_attr;
		pthread_attr_init(&sender_thread_attr);
		pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));
		pthread_create(&sender_thread, &sender_thread_attr, send_actuator, nullptr);
		pthread_attr_destroy(&sender_thread_attr);
	}

	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	PX4_INFO("Got %d from orb_subscribe", _vehicle_status_sub);

	while (!_task_should_exit){

		uint8_t rx_buf[1024];
		//rx_buf[511] = '\0';

		uint64_t timestamp = hrt_absolute_time();

		// Send out sensor messages every 10ms
		// if (got_first_sensor_msg) {
		// 	uint64_t delta_time = timestamp - last_imu_update_timestamp;
		// 	if (delta_time > 15000) {
		// 		PX4_ERR("Sending updates at %llu, delta %llu", timestamp, delta_time);
		// 	}
		// 	uint64_t _px4_gyro_accel_timestamp = hrt_absolute_time();
		// 	_px4_gyro->update(_px4_gyro_accel_timestamp, x_gyro, y_gyro, z_gyro);
		// 	_px4_accel->update(_px4_gyro_accel_timestamp, x_accel, y_accel, z_accel);
		// 	last_imu_update_timestamp = timestamp;
		// }

		// Check for incoming messages from the simulator
		int readRetval = readResponse(&rx_buf[0], sizeof(rx_buf));
		if (readRetval) {
			//Take readRetval and convert it into mavlink msg
			mavlink_message_t msg;
			mavlink_status_t _status{};
			for (int i = 0; i <= readRetval; i++){
				if(mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &_status)){
					//PX4_INFO("Value of msg id: %i", msg.msgid);
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
		}

		bool vehicle_updated = false;
		(void) orb_check(_vehicle_status_sub, &vehicle_updated);
		if (vehicle_updated){
			// PX4_INFO("Value of updated vehicle status: %d", vehicle_updated);
			orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		}

		uint64_t elapsed_time = hrt_absolute_time() - timestamp;
		// if (elapsed_time < 10000) usleep(10000 - elapsed_time);

		if (elapsed_time < 5000) usleep(5000 - elapsed_time);
	}
}

void send_esc_telemetry_dsp(mavlink_hil_actuator_controls_t hil_act_control)
{
	esc_status_s esc_status{};
	esc_status.timestamp = hrt_absolute_time();
	const int max_esc_count = math::min(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS, esc_status_s::CONNECTED_ESC_MAX);

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	int max_esc_index = 0;
	_battery_status_sub.update(&_battery_status);
	for (int i = 0; i < max_esc_count; i++) {
		if (_output_functions[i] != 0) {
			max_esc_index = i;
		}

		esc_status.esc[i].actuator_function = _output_functions[i]; // TODO: this should be in pwm_sim...
		esc_status.esc[i].timestamp = esc_status.timestamp;
		esc_status.esc[i].esc_errorcount = 0; // TODO
		esc_status.esc[i].esc_voltage = _battery_status.voltage_v;
		esc_status.esc[i].esc_current = armed ? 1.0f + math::abs_t(hil_act_control.controls[i]) * 15.0f :
						0.0f; // TODO: magic number
		esc_status.esc[i].esc_rpm = hil_act_control.controls[i] * 6000;  // TODO: magic number
		esc_status.esc[i].esc_temperature = 20.0 + math::abs_t((double)hil_act_control.controls[i]) * 40.0;
	}

	esc_status.esc_count = max_esc_index + 1;
	esc_status.esc_armed_flags = (1u << esc_status.esc_count) - 1;
	esc_status.esc_online_flags = (1u << esc_status.esc_count) - 1;

	_esc_status_pub.publish(esc_status);
}


void
handle_message_command_long_dsp(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	if(debug){
		PX4_INFO("Value of command_long.command: %d", cmd_mavlink.command);
	}

	mavlink_command_ack_t ack = {};
	ack.result = MAV_RESULT_UNSUPPORTED;

	mavlink_message_t ack_message = {};
	mavlink_msg_command_ack_encode(1, 1, &ack_message, &ack);

	uint8_t  acknewBuf[512];
	uint16_t acknewBufLen = 0;
	acknewBufLen = mavlink_msg_to_send_buffer(acknewBuf, &ack_message);
	int writeRetval = writeResponse(&acknewBuf, acknewBufLen);
	PX4_INFO("Succesful write of ACK back over UART: %d at %llu", writeRetval, hrt_absolute_time());
}

void
handle_message_radio_status_dsp(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (MAVLINK_COMM_0 < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status{};

		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		if(debug){
			PX4_INFO("Value of radio status timestamp: %d", status.timestamp);
			PX4_INFO("Value of radio status rssi: %d", status.rssi);
			PX4_INFO("Value of radio status remote_rssi: %d", status.remote_rssi);
			PX4_INFO("Value of radio status txbuf: %d", status.txbuf);
			PX4_INFO("Value of radio status noise: %d", status.noise);
			PX4_INFO("Value of radio status remote_noise: %d", status.remote_noise);
			PX4_INFO("Value of radio status rxerrors: %d", status.rxerrors);
			PX4_INFO("Value of radio status fix: %d", status.fix);
		} else if(!debug && !radio_once){
			PX4_INFO("Value of radio status timestamp: %d", status.timestamp);
			PX4_INFO("Value of radio status rssi: %d", status.rssi);
			PX4_INFO("Value of radio status remote_rssi: %d", status.remote_rssi);
			PX4_INFO("Value of radio status txbuf: %d", status.txbuf);
			PX4_INFO("Value of radio status noise: %d", status.noise);
			PX4_INFO("Value of radio status remote_noise: %d", status.remote_noise);
			PX4_INFO("Value of radio status rxerrors: %d", status.rxerrors);
			PX4_INFO("Value of radio status fix: %d", status.fix);
			radio_once = true;
		}
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

	if(debug){
		PX4_INFO("RC Message received");
	} else if(!debug && !rc_once){
		PX4_INFO("RC Message received (not in debug mode - enable to see when all rc's come in)");
		rc_once = true;
	}
}

void
handle_message_vision_position_estimate_dsp(mavlink_message_t *msg)
{
	return;
	// mavlink_vision_position_estimate_t ev;
	// mavlink_msg_vision_position_estimate_decode(msg, &ev);

	// vehicle_odometry_s visual_odom{};

	// uint64_t timestamp = hrt_absolute_time();

	// visual_odom.timestamp = timestamp;
	// visual_odom.timestamp_sample = timestamp;

	// visual_odom.x = ev.x;
	// visual_odom.y = ev.y;
	// visual_odom.z = ev.z;
	// matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
	// q.copyTo(visual_odom.q);

	// visual_odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// const size_t URT_SIZE = sizeof(visual_odom.pose_covariance) / sizeof(visual_odom.pose_covariance[0]);
	// static_assert(URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
	// 	      "Odometry Pose Covariance matrix URT array size mismatch");

	// for (size_t i = 0; i < URT_SIZE; i++) {
	// 	visual_odom.pose_covariance[i] = ev.covariance[i];
	// }

	// visual_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	// visual_odom.vx = NAN;
	// visual_odom.vy = NAN;
	// visual_odom.vz = NAN;
	// visual_odom.rollspeed = NAN;
	// visual_odom.pitchspeed = NAN;
	// visual_odom.yawspeed = NAN;
	// visual_odom.velocity_covariance[0] = NAN;

	// _visual_odometry_pub.publish(visual_odom);
}

void
handle_message_odometry_dsp(mavlink_message_t *msg)
{
	// mavlink_odometry_t odom;
	// mavlink_msg_odometry_decode(msg, &odom);

	// vehicle_odometry_s odometry{};

	// uint64_t timestamp = hrt_absolute_time();

	// odometry.timestamp = timestamp;
	// odometry.timestamp_sample = timestamp;

	// /* The position is in a local FRD frame */
	// odometry.x = odom.x;
	// odometry.y = odom.y;
	// odometry.z = odom.z;

	// /**
	//  * The quaternion of the ODOMETRY msg represents a rotation from body frame
	//  * to a local frame
	//  */
	// matrix::Quatf q_body_to_local(odom.q);
	// q_body_to_local.normalize();
	// q_body_to_local.copyTo(odometry.q);

	// // pose_covariance
	// static constexpr size_t POS_URT_SIZE = sizeof(odometry.pose_covariance) / sizeof(odometry.pose_covariance[0]);
	// static_assert(POS_URT_SIZE == (sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0])),
	// 	      "Odometry Pose Covariance matrix URT array size mismatch");

	// // velocity_covariance
	// static constexpr size_t VEL_URT_SIZE = sizeof(odometry.velocity_covariance) / sizeof(odometry.velocity_covariance[0]);
	// static_assert(VEL_URT_SIZE == (sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0])),
	// 	      "Odometry Velocity Covariance matrix URT array size mismatch");

	// // TODO: create a method to simplify covariance copy
	// for (size_t i = 0; i < POS_URT_SIZE; i++) {
	// 	odometry.pose_covariance[i] = odom.pose_covariance[i];
	// }

	// /**
	//  * PX4 expects the body's linear velocity in the local frame,
	//  * the linear velocity is rotated from the odom child_frame to the
	//  * local NED frame. The angular velocity needs to be expressed in the
	//  * body (fcu_frd) frame.
	//  */
	// if (odom.child_frame_id == MAV_FRAME_BODY_FRD) {

	// 	odometry.velocity_frame = vehicle_odometry_s::BODY_FRAME_FRD;
	// 	odometry.vx = odom.vx;
	// 	odometry.vy = odom.vy;
	// 	odometry.vz = odom.vz;

	// 	odometry.rollspeed = odom.rollspeed;
	// 	odometry.pitchspeed = odom.pitchspeed;
	// 	odometry.yawspeed = odom.yawspeed;

	// 	for (size_t i = 0; i < VEL_URT_SIZE; i++) {
	// 		odometry.velocity_covariance[i] = odom.velocity_covariance[i];
	// 	}

	// } else {
	// 	PX4_ERR("Body frame %u not supported. Unable to publish velocity", odom.child_frame_id);
	// }

	// /**
	//  * Supported local frame of reference is MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_FRD
	//  * The supported sources of the data/tesimator type are MAV_ESTIMATOR_TYPE_VISION,
	//  * MAV_ESTIMATOR_TYPE_VIO and MAV_ESTIMATOR_TYPE_MOCAP
	//  *
	//  * @note Regarding the local frames of reference, the appropriate EKF_AID_MASK
	//  * should be set in order to match a frame aligned (NED) or not aligned (FRD)
	//  * with true North
	//  */
	// if (odom.frame_id == MAV_FRAME_LOCAL_NED || odom.frame_id == MAV_FRAME_LOCAL_FRD) {

	// 	if (odom.frame_id == MAV_FRAME_LOCAL_NED) {
	// 		odometry.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// 	} else {
	// 		odometry.local_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	// 	}

	// 	if (odom.estimator_type == MAV_ESTIMATOR_TYPE_VISION || odom.estimator_type == MAV_ESTIMATOR_TYPE_VIO) {
	// 		_visual_odometry_pub.publish(odometry);

	// 	} else if (odom.estimator_type == MAV_ESTIMATOR_TYPE_MOCAP) {
	// 		_mocap_odometry_pub.publish(odometry);

	// 	} else {
	// 		PX4_ERR("Estimator source %u not supported. Unable to publish pose and velocity", odom.estimator_type);
	// 	}

	// } else {
	// 	PX4_ERR("Local frame %u not supported. Unable to publish pose and velocity", odom.frame_id);
	// }
	return;
}

void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	if (armed) {
		for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
			msg->controls[i] = _actuator_outputs.output[i];
		}
		//PX4_INFO("Value of actuator data: %f, %f, %f, %f", (double)msg->controls[0], (double)msg->controls[1], (double)msg->controls[2], (double)msg->controls[3]);
	}

	msg->mode = mode_flag_custom;
	msg->mode |= (armed) ? mode_flag_armed : 0;
	msg->flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg->flags |= 1;
#endif
}

int openPort(const char *dev, speed_t speed)
{
	if (_uart_fd >= 0) {
		PX4_ERR("Port in use: %s (%i)", dev, errno);
		return -1;
	}

	_uart_fd = qurt_uart_open(dev, speed);
	PX4_DEBUG("qurt_uart_opened");

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port: %s (%i)", dev, errno);
		return -1;
	}

	return 0;
}

int closePort()
{
	_uart_fd = -1;

	return 0;
}

int readResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for reading or buffer");
		return -1;
	}
    return qurt_uart_read(_uart_fd, (char*) buf, len, ASYNC_UART_READ_WAIT_US);
}

int writeResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for writing or buffer");
		return -1;
	}

    return qurt_uart_write(_uart_fd, (const char*) buf, len);
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("dsp_hitl__main",
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
	PX4_INFO("Usage: dsp_hitl {start|info|stop}");
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
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);
		}

		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}

			_px4_gyro->update(gyro_accel_time, hil_sensor.xgyro, hil_sensor.ygyro, hil_sensor.zgyro);
		}
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);
		}

		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}

			_px4_accel->update(gyro_accel_time, hil_sensor.xacc, hil_sensor.yacc, hil_sensor.zacc);
		}
	}


	// magnetometer
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

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = gyro_accel_time;
		sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		sensor_baro.pressure = hil_sensor.abs_pressure * 100.0f; // hPa to Pa
		sensor_baro.temperature = hil_sensor.temperature;
		sensor_baro.error_count = 0;
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);
	}

	// differential pressure
	if ((hil_sensor.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {
		differential_pressure_s report{};
		report.timestamp_sample = gyro_accel_time;
		report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		report.temperature = hil_sensor.temperature;
		report.differential_pressure_pa = hil_sensor.diff_pressure * 100.0f; // hPa to Pa
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = gyro_accel_time;
		hil_battery_status.voltage_v = 16.0f;
		hil_battery_status.voltage_filtered_v = 16.0f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.connected = true;
		hil_battery_status.remaining = 0.70;
		hil_battery_status.time_remaining_s = NAN;

		_battery_pub.publish(hil_battery_status);
	}
}

uint64_t first_gps_msg_timestamp = 0;
uint64_t first_gps_report_timestamp = 0;

void
handle_message_hil_gps_dsp(mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	sensor_gps_s gps{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 1;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	gps.device_id = device_id.devid;

	gps.lat = hil_gps.lat;
	gps.lon = hil_gps.lon;
	gps.alt = hil_gps.alt;
	gps.alt_ellipsoid = hil_gps.alt;

	gps.s_variance_m_s = 0.25f;
	gps.c_variance_rad = 0.5f;
	gps.fix_type = hil_gps.fix_type;

	gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
	gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

	gps.hdop = 0; // TODO
	gps.vdop = 0; // TODO

	gps.noise_per_ms = 0;
	gps.automatic_gain_control = 0;
	gps.jamming_indicator = 0;
	gps.jamming_state = 0;
	gps.spoofing_state = 0;

	gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
	gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
	gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
	gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
	gps.cog_rad = ((hil_gps.cog == 65535) ? (float)NAN : matrix::wrap_2pi(math::radians(
				hil_gps.cog * 1e-2f))); // cdeg -> rad
	gps.vel_ned_valid = true;

	gps.timestamp_time_relative = 0;
	gps.time_utc_usec = hil_gps.time_usec;

	gps.satellites_used = hil_gps.satellites_visible;

	gps.heading = NAN;
	gps.heading_offset = NAN;

	gps.timestamp = hrt_absolute_time();

	_sensor_gps_pub.publish(gps);

	// sensor_gps_s hil_gps{};
	// const uint64_t timestamp = hrt_absolute_time();

	// hil_gps.timestamp_time_relative = 0;
	// hil_gps.time_utc_usec = gps.time_usec;

	// hil_gps.timestamp = timestamp;
	// hil_gps.lat = gps.lat;
	// hil_gps.lon = gps.lon;
	// hil_gps.alt = gps.alt;
	// hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	// hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	// hil_gps.s_variance_m_s = 0.1f;

	// hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	// hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	// hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	// hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	// hil_gps.vel_ned_valid = true;
	// hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	// hil_gps.fix_type = gps.fix_type;
	// hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	// hil_gps.heading = NAN;
	// hil_gps.heading_offset = NAN;

	// _gps_pub.publish(hil_gps);
}

}
int dsp_hitl_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		dsp_hitl::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return dsp_hitl::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return dsp_hitl::stop();
	}

	else if (!strcmp(verb, "info")) {
		return dsp_hitl::info();
	}

	else {
		dsp_hitl::usage();
		return 1;
	}
}