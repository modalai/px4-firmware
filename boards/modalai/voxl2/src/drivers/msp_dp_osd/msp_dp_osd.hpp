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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "MspDPV1.hpp"
#include <drivers/osd/msp_osd/MessageDisplay/MessageDisplay.hpp>
#include <drivers/osd/msp_osd/uorb_to_msp.hpp>
#include "uorb_to_msp_dp.hpp"
#include "msp_osd_symbols.h"

using namespace time_literals;

// location to "hide" unused display elements
#define LOCATION_HIDDEN 234;

struct PerformanceData {
	bool initialization_problems{false};
	long unsigned int successful_sends{0};
	long unsigned int unsuccessful_sends{0};
};

// mapping from symbol name to bit in the parameter bitmask
//  @TODO investigate params; it seems like this should be available directly?
enum SymbolIndex : uint8_t {
	CRAFT_NAME		= 0,
	DISARMED		= 1,
	GPS_LAT			= 2,
	GPS_LON			= 3,
	GPS_SATS		= 4,
	GPS_SPEED		= 5,
	HOME_DIST		= 6,
	HOME_DIR		= 7,
	MAIN_BATT_VOLTAGE	= 8,
	CURRENT_DRAW		= 9,
	MAH_DRAWN		= 10,
	RSSI_VALUE		= 11,
	ALTITUDE		= 12,
	NUMERICAL_VARIO		= 13,
	FLYMODE			= 14,
	ESC_TMP			= 15,
	PITCH_ANGLE		= 16,
	ROLL_ANGLE		= 17,
	CROSSHAIRS		= 18,
	AVG_CELL_VOLTAGE	= 19,
	HORIZON_SIDEBARS	= 20,
	POWER			= 21
};

class MspDPOsd : public ModuleBase<MspDPOsd>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MspDPOsd(const char *device);

	~MspDPOsd() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;

	// update a single display element in the display
	void Send(const unsigned int message_type, const void *payload, mspDirection_e direction = MSP_DIRECTION_REQUEST);

	// send full configuration to MSP (triggers the actual update)
	void SendConfig();
	void SendTelemetry();

	// perform actions required for local updates
	void parameters_update();

	// convenience function to check if a given symbol is enabled
	bool enabled(const SymbolIndex &symbol);

	MspDPV1 _msp{0};
	int _msp_fd{-1};

	msp_osd::MessageDisplay _display{};

	bool _is_initialized{false};

	// subscriptions to desired vehicle display information
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// local heartbeat
	bool _heartbeat{false};

	typedef struct {
		int32_t		rssi_col;
		int32_t		rssi_row;
		int32_t		current_draw_col;
		int32_t		current_draw_row;
		int32_t		battery_col;
		int32_t		battery_row;
		int32_t		cell_battery_col;
		int32_t		cell_battery_row;
		int32_t		disarmed_col;
		int32_t		disarmed_row;
		int32_t		status_col;
		int32_t		status_row;
		int32_t		flight_mode_col;
		int32_t		flight_mode_row;
		int32_t 	latitude_col;
		int32_t 	latitude_row;
		int32_t 	longitude_col;
		int32_t		longitude_row;
		int32_t		to_home_col;
		int32_t		to_home_row;
		int32_t		crosshair_col;
		int32_t		crosshair_row;
		int32_t		heading_col;
		int32_t		heading_row;
	} msp_dp_osd_params_t;

	// parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::OSD_CH_HEIGHT>) _param_osd_ch_height,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_RSSI_COL>) _param_osd_rssi_col,
		(ParamInt<px4::params::OSD_RSSI_ROW>) _param_osd_rssi_row,
		(ParamInt<px4::params::OSD_CURR_COL>) _param_osd_current_draw_col,
		(ParamInt<px4::params::OSD_CURR_ROW>) _param_osd_current_draw_row,
		(ParamInt<px4::params::OSD_BATT_COL>) _param_osd_batt_col,
		(ParamInt<px4::params::OSD_BATT_ROW>) _param_osd_batt_row,
		(ParamInt<px4::params::OSD_CBATT_COL>) _param_osd_cbatt_col,
		(ParamInt<px4::params::OSD_CBATT_ROW>) _param_osd_cbatt_row,
		(ParamInt<px4::params::OSD_DIS_COL>) _param_osd_disarmed_col,
		(ParamInt<px4::params::OSD_DIS_ROW>) _param_osd_disarmed_row,
		(ParamInt<px4::params::OSD_STATUS_COL>) _param_osd_status_col,
		(ParamInt<px4::params::OSD_STATUS_ROW>) _param_osd_status_row,
		(ParamInt<px4::params::OSD_FM_COL>) _param_osd_flightmode_col,
		(ParamInt<px4::params::OSD_FM_ROW>) _param_osd_flightmode_row,
		(ParamInt<px4::params::OSD_LAT_COL>) _param_osd_lat_col,
		(ParamInt<px4::params::OSD_LAT_ROW>) _param_osd_lat_row,
		(ParamInt<px4::params::OSD_LONG_COL>) _param_osd_long_col,
		(ParamInt<px4::params::OSD_LONG_ROW>) _param_osd_long_row,
		(ParamInt<px4::params::OSD_HOME_COL>) _param_osd_home_col,
		(ParamInt<px4::params::OSD_HOME_ROW>) _param_osd_home_row,
		(ParamInt<px4::params::OSD_CH_COL>) _param_osd_crosshair_col,
		(ParamInt<px4::params::OSD_CH_ROW>) _param_osd_crosshair_row,
		(ParamInt<px4::params::OSD_HDG_COL>) _param_osd_heading_col,
		(ParamInt<px4::params::OSD_HDG_ROW>) _param_osd_heading_row
	)

	// metadata
	msp_dp_osd_params_t	_parameters{0};
	char _device[64] {};
	uint8_t fontType{0};
	resolutionType_e resolution{HD_5018};
	uint8_t row_max[4]{15,17,15,19};
	uint8_t column_max[4]{29,49,29,52};
	PerformanceData _performance_data{};
};

