/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/* Notes:
 *  - Currently there's a lot of wasted processing here if certain displays are enabled.
 *    A relatively low-hanging fruit would be figuring out which display elements require
 *    information from what UORB topics and disable if the information isn't displayed.
 * 	(this is complicated by the fact that it's not a one-to-one mapping...)
 */

#include "msp_osd.hpp"

#include "msp_defines.h"

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/geo/geo.h>

#include "MspV1.hpp"

//OSD elements positions
//in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

// Currently working elements positions (hardcoded)

/* center col

Speed Power Alt
Rssi cell_voltage mah
craft name

*/

// Left
const uint16_t osd_gps_lat_pos = 2048;
const uint16_t osd_gps_lon_pos = 2080;
const uint16_t osd_gps_sats_pos = 2112;

// Center
// Top
const uint16_t osd_disarmed_pos = 2125;
const uint16_t osd_home_dir_pos = 2093;
const uint16_t osd_home_dist_pos = 2095;

// Bottom row 1
const uint16_t osd_gps_speed_pos = 2413;
const uint16_t osd_power_pos = 2415;
const uint16_t osd_altitude_pos = 2416;

// Bottom Row 2
const uint16_t osd_rssi_value_pos = 2445;
const uint16_t osd_avg_cell_voltage_pos = 2446;
const uint16_t osd_mah_drawn_pos = 2449;

// Bottom Row 3
const uint16_t osd_craft_name_pos = 2480;
const uint16_t osd_crosshairs_pos = 2319;

// Right
const uint16_t osd_main_batt_voltage_pos = 2073;
const uint16_t osd_current_draw_pos = 2103;


const uint16_t osd_numerical_vario_pos = LOCATION_HIDDEN;

MspOsd::MspOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);

	// back up device name for connection later
	strcpy(_device, device);

	// _is_initialized = true;
	PX4_INFO("MSP OSD running on %s", _device);
}

MspOsd::~MspOsd()
{
	if(_msp_fd) close(_msp_fd);
}

bool MspOsd::init()
{
	ScheduleOnInterval(100_ms);

	return true;
}


void MspOsd::SendConfig()
{
	PX4_INFO("Sending config full OSD config");

	msp_osd_config_t msp_osd_config;

	msp_osd_config.units = 0;
	msp_osd_config.osd_item_count = 56;
	msp_osd_config.osd_stat_count = 24;
	msp_osd_config.osd_timer_count = 2;
	msp_osd_config.osd_warning_count = 16;              // 16
	msp_osd_config.osd_profile_count = 1;              // 1
	msp_osd_config.osdprofileindex = 1;                // 1
	msp_osd_config.overlay_radio_mode = 0;             //  0

	// display conditional elements
	msp_osd_config.osd_craft_name_pos = enabled(SymbolIndex::CRAFT_NAME) ? osd_craft_name_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_disarmed_pos = enabled(SymbolIndex::DISARMED) ? osd_disarmed_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lat_pos = enabled(SymbolIndex::GPS_LAT) ? osd_gps_lat_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lon_pos = enabled(SymbolIndex::GPS_LON) ? osd_gps_lon_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_sats_pos = enabled(SymbolIndex::GPS_SATS) ? osd_gps_sats_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_speed_pos = enabled(SymbolIndex::GPS_SPEED) ? osd_gps_speed_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dist_pos = enabled(SymbolIndex::HOME_DIST) ? osd_home_dist_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dir_pos = enabled(SymbolIndex::HOME_DIR) ? osd_home_dir_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_voltage_pos = enabled(SymbolIndex::MAIN_BATT_VOLTAGE) ? osd_main_batt_voltage_pos :
			LOCATION_HIDDEN;
	msp_osd_config.osd_current_draw_pos = enabled(SymbolIndex::CURRENT_DRAW) ? osd_current_draw_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_mah_drawn_pos = enabled(SymbolIndex::MAH_DRAWN) ? osd_mah_drawn_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_value_pos = enabled(SymbolIndex::RSSI_VALUE) ? osd_rssi_value_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_altitude_pos = enabled(SymbolIndex::ALTITUDE) ? osd_altitude_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_vario_pos = enabled(SymbolIndex::NUMERICAL_VARIO) ? osd_numerical_vario_pos :
			LOCATION_HIDDEN;

	msp_osd_config.osd_power_pos = enabled(SymbolIndex::POWER) ? osd_power_pos : LOCATION_HIDDEN;
	msp_osd_config.osd_avg_cell_voltage_pos = enabled(SymbolIndex::AVG_CELL_VOLTAGE) ? osd_avg_cell_voltage_pos :
			LOCATION_HIDDEN;

	// the location of our crosshairs can change
	msp_osd_config.osd_crosshairs_pos = LOCATION_HIDDEN;

	if (enabled(SymbolIndex::CROSSHAIRS)) {
		msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos - 32 * _param_osd_ch_height.get();
	}

	// possibly available, but not currently used
	msp_osd_config.osd_flymode_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_tmp_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_horizon_sidebars_pos = 		LOCATION_HIDDEN;

	// Not implemented or not available
	msp_osd_config.osd_artificial_horizon_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_1_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_2_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_throttle_pos_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_vtx_channel_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_yaw_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pidrate_profile_pos =		LOCATION_HIDDEN;
	msp_osd_config.osd_warnings_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_debug_pos = 				LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_usage_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_heading_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_compass_bar_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_remaining_time_estimate_pos = 	LOCATION_HIDDEN;
	msp_osd_config.osd_rtc_datetime_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_adjustment_range_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_core_temperature_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_anti_gravity_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_g_force_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_motor_diag_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_log_status_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flip_arrow_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_link_quality_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flight_dist_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_left_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_right_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_display_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_freq_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rate_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_pid_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_profile_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_dbm_value_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_rc_channels_pos = 			LOCATION_HIDDEN;

	_msp.Send(MSP_OSD_CONFIG, &msp_osd_config);
}

void MspOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		parameters_update();
	}

	// perform first time initialization, if needed
	if (!_is_initialized) {
		struct termios t;
		_msp_fd = open(_device, O_RDWR | O_NONBLOCK);

		if (_msp_fd < 0) {
			_performance_data.initialization_problems = true;
			return;
		}

		tcgetattr(_msp_fd, &t);
		cfsetspeed(&t, B115200);
		t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
		t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
		t.c_oflag = 0;
		tcsetattr(_msp_fd, TCSANOW, &t);

		_msp = MspV1(_msp_fd);

		_is_initialized = true;
		

		// Clear old info on OSD
		PX4_INFO("");
		PX4_INFO("Sending CLEAR CMD");
		const auto clear_osd_msg = msp_osd::construct_OSD_clear();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);

		// Send VTX Config
		PX4_INFO("");
		PX4_INFO("Sending VTX CONFIG");
		const auto vtx_config_msg = msp_osd::construct_vtx_config();
		this->Send(MSP_VTX_CONFIG, &vtx_config_msg, MSP_DIRECTION_REPLY);

		// Send OSD resolution, font 
		PX4_INFO("");
		PX4_INFO("Sending OSD CONFIG");
		const auto osd_config_msg = msp_osd::construct_OSD_config(this->resolution, this->fontType);
		this->Send(MSP_CMD_DISPLAYPORT, &osd_config_msg, MSP_DIRECTION_REPLY);
	
		// Send OSD Canvas size
		PX4_INFO("");
		PX4_INFO("Sending OSD CANVAS");
		const auto osd_canvas_msg = msp_osd::construct_OSD_canvas(row_max[resolution], column_max[resolution]);
		this->Send(MSP_SET_OSD_CANVAS, &osd_canvas_msg, MSP_DIRECTION_REPLY);
	}

	// avoid premature pessimization; if skip processing if we're effectively disabled
	if (_param_osd_symbols.get() == 0) {
		return;
	}

	// Heartbeat
    // a) ensure display is not released by remote OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
	{
		// PX4_INFO("");
		// PX4_INFO("Sending HEARTBEAT");
		const auto heartbeat_msg = msp_osd::construct_OSD_heartbeat();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &heartbeat_msg, MSP_DIRECTION_REPLY);
	}

	// Clear screen
	{
		// PX4_INFO("");
		// PX4_INFO("Sending CLEAR CMD");
		const auto clear_osd_msg = msp_osd::construct_OSD_clear();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);
	}

	// Construct display message...
	// FLIGHT MODE | ARMING | HEADING 
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		log_message_s log_message{};
		_log_message_sub.copy(&log_message);
		const auto display_msg = msp_osd::construct_display_message( vehicle_status, vehicle_attitude, log_message, _param_osd_log_level.get(), _display);
		uint8_t output[sizeof(msp_osd_dp_cmd_t) + sizeof(display_msg.craft_name)+1]{0};	// size of output buffer is size of OSD display port command struct and the buffer you want shown on OSD
		memset(output, 0, sizeof(output));
		msp_osd::construct_OSD_write(column_max[(uint8_t)resolution]/2 - 5, 0, false, display_msg.craft_name, output, sizeof(output));	// col 19, row 0 in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		this->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
	}

	// MSP_FC_VARIANT #1
	{
		// PX4_INFO("");
		// PX4_INFO("Sending FC VARIANT");
		const auto msg = msp_osd::construct_FC_VARIANT();
		this->Send(MSP_FC_VARIANT, &msg, MSP_DIRECTION_REPLY);
	}

	// MSP_STATUS #2 
	{
		// PX4_INFO("");
		// PX4_INFO("Sending MSP STATUS");
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);
		const auto msg = msp_osd::construct_STATUS_HDZ(vehicle_status);
		this->Send(MSP_STATUS, &msg, MSP_DIRECTION_REPLY);
	}

	// MSP RC #3
	{
		// PX4_INFO("");
		// PX4_INFO("Sending RC");
		input_rc_s input_rc{};
		_input_rc_sub.copy(&input_rc);
		const auto msg = msp_osd::construct_RC(input_rc);
		this->Send(MSP_RC, &msg, MSP_DIRECTION_REPLY);
	}
}

void MspOsd::Send(const unsigned int message_type, const void *payload, mspDirection_e direction)
{
	if (_msp.Send(message_type, payload, direction)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}

void MspOsd::parameters_update()
{
	// update our display rate and dwell time
	_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));
}

bool MspOsd::enabled(const SymbolIndex &symbol)
{
	return _param_osd_symbols.get() & (1u << symbol);
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	// initialize device
	const char *device = nullptr;
	bool error_flag = false;

	// loop through input arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	MspOsd *instance = new MspOsd(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MspOsd::print_status()
{
	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tscroll rate: %d", static_cast<int>(_param_osd_scroll_rate.get()));
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);

	// print current display string
	char msg[FULL_MSG_BUFFER];
	_display.get(msg, hrt_absolute_time());
	PX4_INFO("Current message: \n\t%s", msg);

	return 0;
}

int MspOsd::custom_command(int argc, char *argv[])
{	
	int myoptind = 0;
	int ch;
	int row{0};
	int col{0};
	int cmd_fontType{0};
	int cmd_resolution{0};
	char cmd_string[get_instance()->column_max[get_instance()->resolution]]{0};
	const char* resolutions[4] = {"SD_3016", "HD_5018", "HD_3016", "HD_5320"};
	const char *myoptarg = nullptr;
	const char *verb = argv[argc - 1];
	PX4_INFO("Executing the following command: %s", verb);

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;
	}

	while ((ch = px4_getopt(argc, argv, "l:c:f:r:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'l':	// row == line
			row = atoi(myoptarg); // 0 min, 17 max, HD_5018 TRUNCATES MSG
			if (row < 0) row = 0;
			if (row > get_instance()->row_max[get_instance()->resolution]) row = 17;
			PX4_INFO("Got Row: %i", row);
			break;

		case 'c':
			col = atoi(myoptarg);	// 0 min, 49 max, HD_5018 TRUNCATES MSG
			if (col < 0) col = 0;
			if (col > get_instance()->column_max[get_instance()->resolution]) col = 49;
			PX4_INFO("Got Col: %i", col);
			break;

		case 'f':
			cmd_fontType = atoi(myoptarg);
			if (cmd_fontType < 0 || cmd_fontType > 3){
				print_usage("Invalid font type, must be 0-3.");
				return 0;
			}
			PX4_INFO("Got fontType: %i", cmd_fontType);
			break;

		case 'r':
			cmd_resolution = atoi(myoptarg);
			if (cmd_resolution < 0 || cmd_resolution > 3){
				print_usage("Invalid resolution, must be 0-3.");
				return 0;
			}
			PX4_INFO("Got Resolution: %s", resolutions[cmd_resolution]);
			break;

		case 's':
			if (strlen(myoptarg) > MSP_OSD_MAX_STRING_LENGTH){
				PX4_WARN("String length too long, max string length: %i. Message may be truncated.", MSP_OSD_MAX_STRING_LENGTH);
				// print_usage("String length too long, max string length: 30.");
				// return 0;
			}
			PX4_INFO("Got string: %s, Length: %lu", myoptarg, strlen(myoptarg));
			strncpy(cmd_string, myoptarg, strlen(myoptarg) + 1);
			break;

		default:
			print_usage("Unknown command, parsing flags");
			return 0;
		}
	}

	// Write string for testing placement on canvas
	if(!strcmp(verb,"write")){
		PX4_INFO("");
		PX4_INFO("Sending WRITE CMD");
		char line[2];	// 30 char max
		line[0] = 0x90; // Battery FULL symbol
		line[1] = 0; 
		uint8_t output[sizeof(msp_osd_dp_cmd_t) + sizeof(line)];
		msp_osd::construct_OSD_write(col, row, false, line, output, sizeof(output));
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		PX4_INFO("");
		PX4_INFO("Sending DRAW CMD");
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Write string
	if(!strcmp(verb,"write_string")){
		PX4_INFO("");
		PX4_INFO("Sending WRITE STRING CMD");
		if (cmd_string[MSP_OSD_MAX_STRING_LENGTH-1] != '\0') cmd_string[MSP_OSD_MAX_STRING_LENGTH-1] = '\0';

		// Convert string to uppercase, otherwise it will try to print symbols instead
		for(size_t i=0; i<strlen(cmd_string);++i){
			cmd_string[i] = (cmd_string[i] >= 'a' && cmd_string[i] <= 'z') ? cmd_string[i] - 'a' + 'A' : cmd_string[i];
		}

		const char* const_cmd_string = cmd_string;
		uint8_t output[sizeof(msp_osd_dp_cmd_t) + strlen(const_cmd_string)+1]{0};
		PX4_INFO("Output String: %s\tSize of output: %lu", const_cmd_string, sizeof(output));
		msp_osd::construct_OSD_write(col, row, false, const_cmd_string, output, sizeof(output));
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		PX4_INFO("");
		PX4_INFO("Sending DRAW CMD");
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Clear OSD
	if(!strcmp(verb,"clear")){
		PX4_INFO("");
		PX4_INFO("Sending CLEAR CMD");
		const auto msg = msp_osd::construct_OSD_clear();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Release OSD 
	if(!strcmp(verb,"release")){
		PX4_INFO("");
		PX4_INFO("Sending RELEASE CMD");
		const auto msg = msp_osd::construct_OSD_release();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Config OSD resolution, font 
	if(!strcmp(verb,"osd_config")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CONFIG CMD");
		const auto msg = msp_osd::construct_OSD_config((resolutionType_e)cmd_resolution, cmd_fontType);
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		get_instance()->resolution = (resolutionType_e)cmd_resolution;
		get_instance()->fontType = cmd_fontType;
		return 0;
	}

	// Config OSD Canvas size 
	if(!strcmp(verb,"osd_canvas")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CANVAS CMD");
		const auto msg = msp_osd::construct_OSD_canvas(get_instance()->row_max[get_instance()->resolution], get_instance()->column_max[get_instance()->resolution]);
		get_instance()->Send(MSP_SET_OSD_CANVAS, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	PX4_WARN("Unknown command: %s", verb);
	return 0;
}

int MspOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyHS1", "/dev/ttyHS1", "UART port", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear", "DisplayPort command: Clear the OSD.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "DisplayPort command: Clears the display and allows local rendering on the display device based on telemetry information etc.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write_string", "DisplayPort command: Write string to OSD at given location");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, get_instance()->row_max[get_instance()->resolution], "Line/Row to write the string on", false);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, get_instance()->column_max[get_instance()->resolution], "Column to write the string on", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("osd_config", "DisplayPort command: Set OSD font type and resolution.");
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 3, "Resolution to set OSD to.", false);
	PRINT_MODULE_USAGE_PARAM_INT('f', 0, 0, sizeof(get_instance()->resolution)-1, "Font type to use for OSD.", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int msp_osd_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}
