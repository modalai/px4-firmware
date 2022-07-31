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

#include <string>
//#include <string.h>
// #include <pthread.h>
//
#include <px4_platform_common/tasks.h>
// #include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
//
#include <drivers/device/qurt/uart.h>
//
// #include <commander/px4_custom_mode.h>
//
// #include <drivers/drv_pwm_output.h>
// #include <drivers/drv_hrt.h>
//
#include <uORB/uORB.h>
// #include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
// #include <uORB/Subscription.hpp>
// #include <uORB/topics/sensor_gps.h>
// #include <uORB/topics/battery_status.h>
// #include <uORB/topics/differential_pressure.h>
// #include <uORB/topics/actuator_outputs.h>
// #include <uORB/topics/actuator_controls.h>
// #include <uORB/topics/vehicle_control_mode.h>
// #include <uORB/topics/vehicle_status.h>
// #include <uORB/topics/telemetry_status.h>
// #include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/input_rc.h>
// #include <uORB/topics/radio_status.h>
//
// #include <v2.0/mavlink_types.h>
#include <v2.0/standard/mavlink.h>
// #include <v2.0/standard/standard.h>
// #include <v2.0/protocol.h>
// #include <v2.0/mavlink_helpers.h>
// #include <v2.0/minimal/mavlink_msg_heartbeat.h>
//
// #include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
// #include <lib/drivers/barometer/PX4Barometer.hpp>
// #include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
// #include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
//
#include <px4_log.h>
// #include <px4_platform_common/module.h>
//
// #include <uORB/topics/vehicle_control_mode.h>
//
// #include <unistd.h>

#define ASYNC_UART_READ_WAIT_US 2000
#define RC_INPUT_RSSI_MAX	100

extern "C" { __EXPORT int tbs_crossfire_main(int argc, char *argv[]); }

namespace tbs_crossfire
{

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;
bool debug = false;
std::string port = "7";
int baudrate = 115200;

// uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
// uORB::Publication<sensor_gps_s>				_gps_pub{ORB_ID(sensor_gps)};
// uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};
// uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
// uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
uORB::PublicationMulti<input_rc_s>			_rc_pub{ORB_ID(input_rc)};

// // hil_sensor and hil_state_quaternion
// enum SensorSource {
// 	ACCEL		= 0b111,
// 	GYRO		= 0b111000,
// 	MAG		= 0b111000000,
// 	BARO		= 0b1101000000000,
// 	DIFF_PRESS	= 0b10000000000
// };
//
// PX4Accelerometer *_px4_accel{nullptr};
// PX4Barometer *_px4_baro{nullptr};
// PX4Gyroscope *_px4_gyro{nullptr};
// PX4Magnetometer *_px4_mag{nullptr};
//
// hrt_abstime _last_heartbeat_check{0};

// hrt_abstime _heartbeat_type_antenna_tracker{0};
// hrt_abstime _heartbeat_type_gcs{0};
// hrt_abstime _heartbeat_type_onboard_controller{0};
// hrt_abstime _heartbeat_type_gimbal{0};
// hrt_abstime _heartbeat_type_adsb{0};
// hrt_abstime _heartbeat_type_camera{0};
//
// hrt_abstime _heartbeat_component_telemetry_radio{0};
// hrt_abstime _heartbeat_component_log{0};
// hrt_abstime _heartbeat_component_osd{0};
// hrt_abstime _heartbeat_component_obstacle_avoidance{0};
// hrt_abstime _heartbeat_component_visual_inertial_odometry{0};
// hrt_abstime _heartbeat_component_pairing_manager{0};
// hrt_abstime _heartbeat_component_udp_bridge{0};
// hrt_abstime _heartbeat_component_uart_bridge{0};

// bool got_first_sensor_msg = false;
// float x_accel = 0;
// float y_accel = 0;
// float z_accel = 0;
// float x_gyro = 0;
// float y_gyro = 0;
// float z_gyro = 0;
// uint64_t gyro_accel_time = 0;
//
// bool _use_software_mav_throttling{false};
// bool _radio_status_available{false};
// bool _radio_status_critical{false};
// float _radio_status_mult{1.0f};

// vehicle_status_s _vehicle_status{};
// vehicle_control_mode_s _control_mode{};
// actuator_outputs_s _actuator_outputs{};
// actuator_controls_s _actuator_controls{};
// radio_status_s _rstatus {};

int openPort(const char *dev, speed_t speed);
int closePort();

int readResponse(void *buf, size_t len);
// int writeResponse(void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
// int info();
bool isOpen() { return _uart_fd >= 0; };

void usage();
void task_main(int argc, char *argv[]);

// void *send_actuator(void *);
// void send_actuator_data();

// void handle_message_hil_sensor_dsp(mavlink_message_t *msg);
// void handle_message_hil_gps_dsp(mavlink_message_t *msg);
// void handle_message_odometry_dsp(mavlink_message_t *msg);
// void handle_message_vision_position_estimate_dsp(mavlink_message_t *msg);
void handle_message_rc_channels_override_dsp(mavlink_message_t *msg);
// void handle_message_radio_status_dsp(mavlink_message_t *msg);
// void handle_message_command_long_dsp(mavlink_message_t *msg);

// void CheckHeartbeats(const hrt_abstime &t, bool force);
void handle_message_dsp(mavlink_message_t *msg);
// void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg);

void
handle_message_dsp(mavlink_message_t *msg)
{
	// //PX4_INFO("msg ID: %d", msg->msgid);
	// // PX4_ERR("msg ID: %d", msg->msgid);
	// switch (msg->msgid) {
	// case MAVLINK_MSG_ID_HIL_SENSOR:
	// 	handle_message_hil_sensor_dsp(msg);
	// 	//PX4_INFO("MAVLINK HIL SENSOR");
	// 	break;
	// case MAVLINK_MSG_ID_HIL_GPS:
	// 	if(!vio){
	// 		handle_message_hil_gps_dsp(msg);
	// 		//PX4_INFO("MAVLINK HIL GPS");
	// 		break;
	// 	} else {
	// 		break;
	// 	}
	// case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
	// 	if(vio){
	// 		handle_message_vision_position_estimate_dsp(msg);
	// 		break;
	// 	}
	// case MAVLINK_MSG_ID_ODOMETRY:
	// 	if(vio){
	// 		handle_message_odometry_dsp(msg);
	// 		break;
	// 	}
	// case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
	// 	handle_message_rc_channels_override_dsp(msg);
	// 	break;
	// case MAVLINK_MSG_ID_RADIO_STATUS:
	// 	handle_message_radio_status_dsp(msg);
	// 	break;
	// case MAVLINK_MSG_ID_COMMAND_LONG:
	// 	handle_message_command_long_dsp(msg);
	// 	break;
	// case MAVLINK_MSG_ID_HEARTBEAT:
	// 	PX4_INFO("Heartbeat msg received");
	// 	break;
	// case MAVLINK_MSG_ID_SYSTEM_TIME:
	// 	// PX4_INFO("MAVLINK SYSTEM TIME");
	// 	break;
	// default:
	// 	PX4_ERR("Unknown msg ID: %d", msg->msgid);
	// 	break;
	// }
}

void task_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "dp:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			debug = true;
			break;
		case 'p':
			port = myoptarg;
			break;
		case 'b':
			baudrate = atoi(myoptarg);
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

	// Create a thread for sending data to the simulator.
	// if(!crossfire){
	// 	pthread_t sender_thread;
	// 	pthread_attr_t sender_thread_attr;
	// 	pthread_attr_init(&sender_thread_attr);
	// 	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));
	// 	pthread_create(&sender_thread, &sender_thread_attr, send_actuator, nullptr);
	// 	pthread_attr_destroy(&sender_thread_attr);
	// }

	// int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	// PX4_INFO("Got %d from orb_subscribe", _vehicle_status_sub);
	// int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls));
	// PX4_INFO("Got %d from orb_subscribe", _actuator_controls_sub);

	// bool vehicle_updated = false;
	// (void) orb_check(_vehicle_status_sub, &vehicle_updated);

	while (!_task_should_exit){

		uint8_t rx_buf[1024];
		//rx_buf[511] = '\0';

		// // Send out sensor messages every 10ms
		// if (got_first_sensor_msg) {
		// 	uint64_t delta_time = timestamp - last_imu_update_timestamp;
		// 	if (delta_time > 15000) {
		// 		PX4_ERR("Sending updates at %llu, delta %llu", timestamp, delta_time);
		// 	}
		// 	uint64_t _px4_gyro_accle_timestamp = hrt_absolute_time();
		// 	_px4_gyro->update(_px4_gyro_accle_timestamp, x_gyro, y_gyro, z_gyro);
		// 	_px4_accel->update(_px4_gyro_accle_timestamp, x_accel, y_accel, z_accel);
		// 	last_imu_update_timestamp = timestamp;
		// }

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

		usleep(500);
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
	}
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

	_task_handle = px4_task_spawn_cmd("tbs_crossfire_main",
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
	PX4_INFO("Usage: tbs_crossfire {start|info|stop}");
}

}
int tbs_crossfire_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		tbs_crossfire::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return tbs_crossfire::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return tbs_crossfire::stop();
	}

	else if (!strcmp(verb, "info")) {
		return tbs_crossfire::info();
	}

	else {
		tbs_crossfire::usage();
		return 1;
	}
}
