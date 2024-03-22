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

#include "msp_dp_osd.hpp"

#include "msp_dp_defines.h"
#include <drivers/osd/msp_osd/msp_defines.h>

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

#include "MspDPV1.hpp"

bool clear{true};

MspDPOsd::MspDPOsd(const char *device) :
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

MspDPOsd::~MspDPOsd()
{
	if(_msp_fd) close(_msp_fd);
}

bool MspDPOsd::init()
{
	ScheduleOnInterval(100_ms);

	return true;
}


void MspDPOsd::Run()
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

		_msp = MspDPV1(_msp_fd);

		_is_initialized = true;
		

		// Clear old info on OSD
		PX4_INFO("");
		PX4_INFO("Sending CLEAR CMD");
		const auto clear_osd_msg = msp_dp_osd::construct_OSD_clear();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);

		// Send VTX Config
		PX4_INFO("");
		PX4_INFO("Sending VTX CONFIG");
		const auto vtx_config_msg = msp_dp_osd::construct_vtx_config();
		this->Send(MSP_VTX_CONFIG, &vtx_config_msg, MSP_DIRECTION_REPLY);

		// Send OSD resolution, font 
		PX4_INFO("");
		PX4_INFO("Sending OSD CONFIG");
		const auto osd_config_msg = msp_dp_osd::construct_OSD_config(this->resolution, this->fontType);
		this->Send(MSP_CMD_DISPLAYPORT, &osd_config_msg, MSP_DIRECTION_REPLY);
	
		// Send OSD Canvas size
		PX4_INFO("");
		PX4_INFO("Sending OSD CANVAS");
		const auto osd_canvas_msg = msp_dp_osd::construct_OSD_canvas(row_max[resolution], column_max[resolution]);
		this->Send(MSP_SET_OSD_CANVAS, &osd_canvas_msg, MSP_DIRECTION_REPLY);
	}

	// avoid premature pessimization; if skip processing if we're effectively disabled
	if (_param_osd_symbols.get() == 0) {
		return;
	}

	const auto osd_config_msg = msp_dp_osd::construct_OSD_config(this->resolution, this->fontType);
	this->Send(MSP_CMD_DISPLAYPORT, &osd_config_msg, MSP_DIRECTION_REPLY);

	// Heartbeat
    // a) ensure display is not released by remote OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
	{
		const auto heartbeat_msg = msp_dp_osd::construct_OSD_heartbeat();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &heartbeat_msg, MSP_DIRECTION_REPLY);
	}

	// Clear screen
	if (clear){
		const auto clear_osd_msg = msp_dp_osd::construct_OSD_clear();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &clear_osd_msg, MSP_DIRECTION_REPLY);
	}

	// FC VARIANT
	{
		const auto msg = msp_osd::construct_FC_VARIANT();
		this->Send(MSP_FC_VARIANT, &msg, MSP_DIRECTION_REPLY);
	}

	// VEHICLE STATUS / DISARMED / ERROR MESSAGES / FLIGHT MODE
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		log_message_s log_message{};
		_log_message_sub.copy(&log_message);

		// Vehicle Status
		const auto msg = msp_dp_osd::construct_STATUS_HDZ(vehicle_status);
		this->Send(MSP_STATUS, &msg, MSP_DIRECTION_REPLY);

		// DISARMED Message -> BOTTOM-MIDDLE TOP
		if(vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED){
			const char* disarmed_msg = "DISARMED";
			uint8_t disarmed_output[sizeof(msp_osd_dp_cmd_t) + sizeof(disarmed_msg)+1]{0};	// size of output buffer is size of OSD display port command struct and the buffer you want shown on OSD
			msp_dp_osd::construct_OSD_write(_parameters.disarmed_col, _parameters.disarmed_row, false, disarmed_msg, disarmed_output, sizeof(disarmed_output));	
			this->Send(MSP_CMD_DISPLAYPORT, &disarmed_output, MSP_DIRECTION_REPLY);
		}

		// STATUS MESSAGE -> BOTTOM-MIDDLE MIDDLE (PX4 error messages)
		// FLIGHT MODE | ARMING | HEADING ....  WILL PRINT PX4 ERROR MESSAGES FOR 30 SECONDS THEN RESET to above format
		const auto display_msg = msp_dp_osd::construct_display_message(vehicle_status, vehicle_attitude, log_message, _param_osd_log_level.get(), _display);
		uint8_t display_msg_output[sizeof(msp_osd_dp_cmd_t) + sizeof(display_msg.craft_name)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.status_col, _parameters.status_row, false, display_msg.craft_name, display_msg_output, sizeof(display_msg_output));	// display_msg max size (w/o warning) is 15
		this->Send(MSP_CMD_DISPLAYPORT, &display_msg_output, MSP_DIRECTION_REPLY);

		// Flight Mode -> BOTTOM-MIDDLE BOTTOM
		const auto flight_mode = msp_dp_osd::construct_flight_mode(vehicle_status);
		uint8_t flight_mode_output[sizeof(msp_osd_dp_cmd_t) + sizeof(flight_mode)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.flight_mode_col, _parameters.flight_mode_row, false, flight_mode, flight_mode_output, sizeof(flight_mode_output));	
		this->Send(MSP_CMD_DISPLAYPORT, &flight_mode_output, MSP_DIRECTION_REPLY);


	}

	// RC CHANNELS / RSSI
	{
		input_rc_s input_rc{};
		_input_rc_sub.copy(&input_rc);

		// Send RC channel values
		const auto rc_msg = msp_dp_osd::construct_RC(input_rc);
		this->Send(MSP_RC, &rc_msg, MSP_DIRECTION_REPLY);

		// Send RSSI
		char rssi[5];
		snprintf(rssi, sizeof(rssi), "%c%d", SYM_RSSI, (int)input_rc.rssi_dbm);
		uint8_t rssi_output[sizeof(msp_osd_dp_cmd_t) + sizeof(rssi)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.rssi_col, _parameters.rssi_row, false, rssi, rssi_output, sizeof(rssi_output));	
		this->Send(MSP_CMD_DISPLAYPORT, &rssi_output, MSP_DIRECTION_REPLY);
	}

	// BATTERY / CURRENT DRAW
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		// Full battery voltage
		char batt[8];
		uint8_t battery_symbol = SYM_BATT_FULL;
		snprintf(batt, sizeof(batt), "%c%.2fV", battery_symbol, static_cast<double>(battery_status.voltage_v));
		uint8_t battery_output[sizeof(msp_osd_dp_cmd_t) + sizeof(batt)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.battery_col, _parameters.battery_row, false, batt, battery_output, sizeof(battery_output));	// col 0, row 17 (bottom left) in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &battery_output, MSP_DIRECTION_REPLY);

		// Per cell battery voltage
		char batt_cell[7];
		// uint8_t battery_symbol = SYM_BATT_FULL;
		snprintf(batt_cell, sizeof(batt_cell), "%c%.2fV", battery_symbol, static_cast<double>(battery_status.voltage_v/battery_status.cell_count));
		uint8_t batt_cell_output[sizeof(msp_osd_dp_cmd_t) + sizeof(batt_cell)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.cell_battery_col, _parameters.cell_battery_row, false, batt_cell, batt_cell_output, sizeof(batt_cell_output));	// col BATT + 1 SPACE, row 17 (bottom left) in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &batt_cell_output, MSP_DIRECTION_REPLY);

		// Current draw
		char current_draw[7];
		snprintf(current_draw, sizeof(current_draw), "%.3f%c", static_cast<double>(battery_status.current_filtered_a), SYM_AMP);
		uint8_t current_draw_output[sizeof(msp_osd_dp_cmd_t) + sizeof(current_draw)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.current_draw_col, _parameters.current_draw_row, false, current_draw, current_draw_output, sizeof(current_draw_output));	// col Max column-sizeof(current_draw_messge), row 0 (top right BOTTOM) in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &current_draw_output, MSP_DIRECTION_REPLY);
	}

	// GPS LAT/LONG
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);
		
		// GPS Longitude 
		char longitude[11];
		snprintf(longitude, sizeof(longitude), "%c%.7f", SYM_LON, static_cast<double>(vehicle_gps_position.lon));
		uint8_t longitude_output[sizeof(msp_osd_dp_cmd_t) + sizeof(longitude)+1]{0};	// size of battery_output buffer is size of OSD display port command struct and the buffer you want shown on OSD
		msp_dp_osd::construct_OSD_write(_parameters.longitude_col, _parameters.longitude_row, false, longitude, longitude_output, sizeof(longitude_output));	// col X, row 15 (bottom right TOP) in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &longitude_output, MSP_DIRECTION_REPLY);

		// GPS Latitude
		char latitude[11];
		snprintf(latitude, sizeof(latitude), "%c%.7f", SYM_LAT, static_cast<double>(vehicle_gps_position.lat));
		uint8_t latitude_output[sizeof(msp_osd_dp_cmd_t) + sizeof(latitude)+1]{0};	// size of battery_output buffer is size of OSD display port command struct and the buffer you want shown on OSD
		msp_dp_osd::construct_OSD_write(_parameters.latitude_col, _parameters.latitude_row, false, latitude, latitude_output, sizeof(latitude_output));	// col X, row 16 (bottom right BOTTOM) in HD_5018
		this->Send(MSP_CMD_DISPLAYPORT, &latitude_output, MSP_DIRECTION_REPLY);
	}

	// DIR/DIST TO HOME
	{
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		estimator_status_s estimator_status{};
		_estimator_status_sub.copy(&estimator_status);
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		int16_t distance_to_home{0};
		int16_t bearing_to_home{SYM_ARROW_NORTH};

		// Calculate distance and direction to home
		if (home_position.valid_hpos && home_position.valid_lpos && estimator_status.solution_status_flags & (1 << 4)) {
			float bearing_to_home_f = math::degrees(get_bearing_to_next_waypoint(vehicle_global_position.lat,
								vehicle_global_position.lon,
								home_position.lat, home_position.lon));

			if (bearing_to_home < 0) {
				bearing_to_home += 360.0f;
			}

			float distance_to_home_f = get_distance_to_next_waypoint(vehicle_global_position.lat,
						vehicle_global_position.lon,
						home_position.lat, home_position.lon);

			distance_to_home = (int16_t)distance_to_home_f; // meters
			bearing_to_home = msp_dp_osd::get_symbol_from_bearing((double)bearing_to_home_f);
		} 
	
		char to_home[8];	// Direction symbol [1 byte], Distance to home [5 bytes max], Meter symbol [1 byte] 
		snprintf(to_home, sizeof(to_home), "%c%i%c", bearing_to_home, distance_to_home, SYM_M);
		uint8_t to_home_output[sizeof(msp_osd_dp_cmd_t) + sizeof(to_home)+1]{0};	
		msp_dp_osd::construct_OSD_write(_parameters.to_home_col, _parameters.to_home_row, false, to_home, to_home_output, sizeof(to_home_output));	
		this->Send(MSP_CMD_DISPLAYPORT, &to_home_output, MSP_DIRECTION_REPLY);
	}

	// // EXPERIMENTATION
	// {
	// 	char experiment[17] = {
	// 		SYM_ARROW_SOUTH, SYM_ARROW_2, SYM_ARROW_3, SYM_ARROW_4,
	// 		SYM_ARROW_EAST, SYM_ARROW_6, SYM_ARROW_7, SYM_ARROW_8,
	// 		SYM_ARROW_NORTH, SYM_ARROW_10, SYM_ARROW_11, SYM_ARROW_12,
	// 		SYM_ARROW_WEST, SYM_ARROW_14, SYM_ARROW_15, SYM_ARROW_16,
	// 	};
	// 	uint8_t experiment_output[sizeof(msp_osd_dp_cmd_t) + sizeof(experiment)+1]{0};	
	// 	msp_dp_osd::construct_OSD_write(8, 10, false, experiment, experiment_output, sizeof(experiment_output));	
	// 	this->Send(MSP_CMD_DISPLAYPORT, &experiment_output, MSP_DIRECTION_REPLY);	
	// }

	// DRAW whole screen
	displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
	this->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
}

void MspDPOsd::Send(const unsigned int message_type, const void *payload, mspDirection_e direction)
{
	if (_msp.Send(message_type, payload, direction)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}

void MspDPOsd::parameters_update()
{
	// update our display rate and dwell time
	_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));

	// Get DisplayPort based positions
	param_get(param_find("OSD_RSSI_COL"),  	&_parameters.rssi_col);
	param_get(param_find("OSD_RSSI_ROW"),  	&_parameters.rssi_row);

	param_get(param_find("OSD_CURR_COL"),  	&_parameters.current_draw_col);
	param_get(param_find("OSD_CURR_ROW"),  	&_parameters.current_draw_row);

	param_get(param_find("OSD_BATT_COL"),  	&_parameters.battery_col);
	param_get(param_find("OSD_BATT_ROW"),  	&_parameters.battery_row);
	param_get(param_find("OSD_CBATT_COL"), 	&_parameters.cell_battery_col);
	param_get(param_find("OSD_CBATT_ROW"), 	&_parameters.cell_battery_row);

	param_get(param_find("OSD_DIS_COL"),  	&_parameters.disarmed_col);
	param_get(param_find("OSD_DIS_ROW"),  	&_parameters.disarmed_row);
	param_get(param_find("OSD_STATUS_COL"), &_parameters.status_col);
	param_get(param_find("OSD_STATUS_ROW"), &_parameters.status_row);
	param_get(param_find("OSD_FM_COL"),  	&_parameters.flight_mode_col);
	param_get(param_find("OSD_FM_ROW"),  	&_parameters.flight_mode_row);

	param_get(param_find("OSD_LAT_COL"),  	&_parameters.latitude_col);
	param_get(param_find("OSD_LAT_ROW"),  	&_parameters.latitude_row);
	param_get(param_find("OSD_LONG_COL"), 	&_parameters.longitude_col);
	param_get(param_find("OSD_LONG_ROW"), 	&_parameters.longitude_row);

	param_get(param_find("OSD_HOME_COL"), 	&_parameters.to_home_col);
	param_get(param_find("OSD_HOME_ROW"), 	&_parameters.to_home_row);
}

bool MspDPOsd::enabled(const SymbolIndex &symbol)
{
	return _param_osd_symbols.get() & (1u << symbol);
}

int MspDPOsd::task_spawn(int argc, char *argv[])
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

	MspDPOsd *instance = new MspDPOsd(device);

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

int MspDPOsd::print_status()
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

// Ex: msp_osd -c 5 -l 5 -s TEST write_string -> Write "TEST" at column 5/row 5 
int MspDPOsd::custom_command(int argc, char *argv[])
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
		line[0] = SYM_BATT_FULL; // Battery FULL symbol
		line[1] = 0; 
		uint8_t output[sizeof(msp_osd_dp_cmd_t) + sizeof(line)]{0};
		msp_dp_osd::construct_OSD_write(col, row, false, line, output, sizeof(output));
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
		// for(size_t i=0; i<strlen(cmd_string);++i){
		// 	cmd_string[i] = (cmd_string[i] >= 'a' && cmd_string[i] <= 'z') ? cmd_string[i] - 'a' + 'A' : cmd_string[i];
		// }

		const char* const_cmd_string = cmd_string;
		uint8_t output[sizeof(msp_osd_dp_cmd_t) + strlen(const_cmd_string)+1]{0};
		PX4_INFO("Output String: %s\tSize of output: %lu", const_cmd_string, sizeof(output));
		msp_dp_osd::construct_OSD_write(col, row, false, const_cmd_string, output, sizeof(output));
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &output, MSP_DIRECTION_REPLY);
		PX4_INFO("");
		PX4_INFO("Sending DRAW CMD");
		displayportMspCommand_e draw{MSP_DP_DRAW_SCREEN};
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &draw, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Turn auto clear OSD on/off 
	if(!strcmp(verb,"toggle_clear")){
		PX4_INFO("");
		PX4_INFO("Sending TOGGLING CLEAR");
		clear = !clear;
		return 0;
	}

	// Release OSD 
	if(!strcmp(verb,"release")){
		PX4_INFO("");
		PX4_INFO("Sending RELEASE CMD");
		const auto msg = msp_dp_osd::construct_OSD_release();
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Config OSD resolution, font 
	if(!strcmp(verb,"osd_config")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CONFIG CMD");
		const auto msg = msp_dp_osd::construct_OSD_config((resolutionType_e)cmd_resolution, cmd_fontType);
		get_instance()->Send(MSP_CMD_DISPLAYPORT, &msg, MSP_DIRECTION_REPLY);
		get_instance()->resolution = (resolutionType_e)cmd_resolution;
		get_instance()->fontType = cmd_fontType;
		return 0;
	}

	// Config OSD Canvas size 
	if(!strcmp(verb,"osd_canvas")){
		PX4_INFO("");
		PX4_INFO("Sending OSD CANVAS CMD");
		const auto msg = msp_dp_osd::construct_OSD_canvas(get_instance()->row_max[get_instance()->resolution], get_instance()->column_max[get_instance()->resolution]);
		get_instance()->Send(MSP_SET_OSD_CANVAS, &msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	// Config VTX settings  
	if(!strcmp(verb,"vtx")){
		/* Fields:
		protocol: 5 -> MSP
		band:	  5 -> RC Band R (A,B,E,F,R)
		channel:  1 -> Channel (Ex: R1, R2, F1, etc)
		power:	  1 -> 0 (0mw), 1 (25mW), 2 (200mW)
		pit:	  0 -> Pit mode off 
		freq:	  0x161A -> 5658 MHz
		*/
		uint8_t protocol{5};
		uint8_t band{5};
		uint8_t channel{1};
		uint8_t power{1};
		uint8_t pit{0};
		uint16_t freq{0x161A};
		PX4_INFO("");
		PX4_INFO("Sending VTX CONFIG");
		// const auto vtx_config_msg = msp_dp_osd::construct_vtx_config();
		const msp_vtx_config_t vtx_config_msg = {
			protocol,
			band,
			channel,
			power,
			pit,
			freq
		};
		get_instance()->Send(MSP_VTX_CONFIG, &vtx_config_msg, MSP_DIRECTION_REPLY);
		return 0;
	}

	PX4_WARN("Unknown command: %s", verb);
	return 0;
}

int MspDPOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP DisplayPort telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_dp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_dp_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyHS1", "/dev/ttyHS1", "UART port", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear", "DisplayPort command: Clear the OSD.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "DisplayPort command: Clears the display and allows local rendering on the display device based on telemetry information etc.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write_string", "DisplayPort command: Write string to OSD at given location");
	// PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, get_instance()->row_max[get_instance()->resolution], "Line/Row to write the string on", false);
	// PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, get_instance()->column_max[get_instance()->resolution], "Column to write the string on", false);
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 19, "Line/Row to write the string on", false);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 52, "Column to write the string on", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("osd_config", "DisplayPort command: Set OSD font type and resolution.");
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 3, "Resolution to set OSD to.", false);
	// PRINT_MODULE_USAGE_PARAM_INT('f', 0, 0, sizeof(get_instance()->resolution)-1, "Font type to use for OSD.", false);
	PRINT_MODULE_USAGE_PARAM_INT('f', 0, 0, 3, "Font type to use for OSD.", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int msp_dp_osd_main(int argc, char *argv[])
{
	return MspDPOsd::main(argc, argv);
}
