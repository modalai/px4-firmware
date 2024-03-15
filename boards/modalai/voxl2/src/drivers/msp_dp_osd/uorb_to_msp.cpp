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

/* uorb_to_msp.cpp
 *
 * Implementation file for UORB -> MSP conversion functions.
 */

// includes for mathematical manipulation
#include <math.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

// clock access
#include <px4_platform_common/defines.h>
using namespace time_literals;

#include "uorb_to_msp.hpp"

namespace msp_dp_osd
{

msp_name_t construct_display_message(const vehicle_status_s &vehicle_status,
				     const vehicle_attitude_s &vehicle_attitude,
				     const log_message_s &log_message,
				     const int log_level,
				     MessageDisplay &display)
{
	// initialize result
	msp_name_t display_message {0};

	const auto now = hrt_absolute_time();
	static uint64_t last_warning_stamp {0};

	// update arming state, flight mode, and warnings, if current
	if (vehicle_status.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::ARMING, "???");
		display.set(MessageDisplayType::FLIGHT_MODE, "???");

	} else {
		// display armed / disarmed
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			display.set(MessageDisplayType::ARMING, "ARM");

		} else {
			display.set(MessageDisplayType::ARMING, "DSRM");
		}

		// display flight mode
		switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			display.set(MessageDisplayType::FLIGHT_MODE, "MANUAL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			display.set(MessageDisplayType::FLIGHT_MODE, "ALTCTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			display.set(MessageDisplayType::FLIGHT_MODE, "POSCTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_MISSION");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_LOITER");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_RTL");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED:
			display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			display.set(MessageDisplayType::FLIGHT_MODE, "ACRO");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED1:
			display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED1");
			break;

		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			display.set(MessageDisplayType::FLIGHT_MODE, "DESCEND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			display.set(MessageDisplayType::FLIGHT_MODE, "TERMINATION");
			break;

		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			display.set(MessageDisplayType::FLIGHT_MODE, "OFFBOARD");
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			display.set(MessageDisplayType::FLIGHT_MODE, "STAB");
			break;

		case vehicle_status_s::NAVIGATION_STATE_UNUSED2:
			display.set(MessageDisplayType::FLIGHT_MODE, "UNUSED2");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_TAKEOFF");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_LAND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_FOLLOW_TARGET");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_PRECLAND");
			break;

		case vehicle_status_s::NAVIGATION_STATE_ORBIT:
			display.set(MessageDisplayType::FLIGHT_MODE, "ORBIT");
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			display.set(MessageDisplayType::FLIGHT_MODE, "AUTO_VTOL_TAKEOFF");
			break;

		case vehicle_status_s::NAVIGATION_STATE_MAX:
			display.set(MessageDisplayType::FLIGHT_MODE, "MAX");
			break;

		default:
			display.set(MessageDisplayType::FLIGHT_MODE, "???");
		}
	}

	// display, if updated
	if (log_message.severity <= log_level) {
		// warn_to_upper(log_message.text);
		display.set(MessageDisplayType::WARNING, log_message.text);
		last_warning_stamp = now;

	} else if (now - last_warning_stamp > 30_s) {
		// clear warning after timeout
		display.set(MessageDisplayType::WARNING, "");
		last_warning_stamp = now;
	}

	// update heading, if relatively recent
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::HEADING, "N?");

	} else {
		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
		const auto yaw = math::degrees(euler_attitude.psi());

		// display north direction
		if (yaw <= 22.5f) {
			display.set(MessageDisplayType::HEADING, "N");

		} else if (yaw <= 67.5f) {
			display.set(MessageDisplayType::HEADING, "NE");

		} else if (yaw <= 112.5f) {
			display.set(MessageDisplayType::HEADING, "E");

		} else if (yaw <= 157.5f) {
			display.set(MessageDisplayType::HEADING, "SE");

		} else if (yaw <= 202.5f) {
			display.set(MessageDisplayType::HEADING, "S");

		} else if (yaw <= 247.5f) {
			display.set(MessageDisplayType::HEADING, "SW");

		} else if (yaw <= 292.5f) {
			display.set(MessageDisplayType::HEADING, "W");

		} else if (yaw <= 337.5f) {
			display.set(MessageDisplayType::HEADING, "NW");

		} else if (yaw <= 360.0f) {
			display.set(MessageDisplayType::HEADING, "N");
		}
	}

	// update message and return
	display.get(display_message.craft_name, hrt_absolute_time());
	return display_message;
}

msp_fc_variant_t construct_FC_VARIANT()
{
	// initialize result
	msp_fc_variant_t variant{};

	memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
	return variant;
}

// New code for HDZero VTX

msp_vtx_config_t construct_vtx_config(){
	msp_vtx_config_t vtx_config {0};

	vtx_config.protocol = 5; 		// MSP
	vtx_config.band 	= 5; 		// BAND 5
	vtx_config.channel 	= 1; 		// CHANNEL 1
	// vtx_config.channel 	= 2; 		// CHANNEL 2
	vtx_config.power 	= 1; 		// POWER LEVEL 1 -> 25mW
	vtx_config.pit	 	= 0; 		// PIT MODE OFF
	vtx_config.freq 	= 0x161A;	// 5865 MHz

	return vtx_config;
}

msp_status_HDZ_t construct_STATUS_HDZ(const vehicle_status_s &vehicle_status){
	msp_status_HDZ_t status_HDZ = {0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status_HDZ.armed = 0x01;
	}

	status_HDZ.arming_disable_flags_count = 1;
	status_HDZ.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	return status_HDZ;
}

msp_rc_t construct_RC(const input_rc_s &input_rc){
	msp_rc_t msp_rc{0};
	
	for (int i=0; i < MSP_MAX_SUPPORTED_CHANNELS; ++i){
		msp_rc.channelValue[i] = input_rc.values[i];
	}
	uint16_t throttle_swap{msp_rc.channelValue[2]};
	msp_rc.channelValue[2] = msp_rc.channelValue[3];	// Ch3 throttle -> yaw
	msp_rc.channelValue[3] = throttle_swap;				// Ch4 yaw -> throttle
	return msp_rc;
}

msp_osd_canvas_t construct_OSD_canvas(uint8_t row, uint8_t col){
	msp_osd_canvas_t msp_canvas{0};

	// HD
	if (row > 49) row = 49;
	if (col > 17) col = 17;
	msp_canvas.row_max = row;
	msp_canvas.col_max = col;

	// msp_canvas.row_max = HD_ROW_MAX;
	// msp_canvas.col_max = HD_COL_MAX;

	// SD
	// msp_canvas.row_max = SD_COL_MAX;
	// msp_canvas.col_max = SD_ROW_MAX;

	return msp_canvas;
}

// Construct a HDZero OSD heartbeat command
displayportMspCommand_e construct_OSD_heartbeat(){
	return MSP_DP_HEARTBEAT;
}

// Construct a HDZero OSD release command
displayportMspCommand_e construct_OSD_release(){
	return MSP_DP_RELEASE;
}

// Construct a HDZero OSD clear command
displayportMspCommand_e construct_OSD_clear(){
	return MSP_DP_CLEAR_SCREEN;
}

// Construct a HDZero OSD write command into an output buffer given location, string, and # bytes to write 
// WARNING: If input string has lowercase chars, they may be interpreted as symbols!
uint8_t construct_OSD_write(uint8_t col, uint8_t row, bool blink, const char *string, uint8_t *output, uint8_t len)
{
	msp_osd_dp_cmd_t msp_osd_dp_cmd;
	int str_len = strlen(string);
    if (str_len > MSP_OSD_MAX_STRING_LENGTH) str_len = MSP_OSD_MAX_STRING_LENGTH;
	msp_osd_dp_cmd.subcmd = (uint8_t)MSP_DP_WRITE_STRING;
	msp_osd_dp_cmd.row = row;
	msp_osd_dp_cmd.col = col;
	msp_osd_dp_cmd.attr = blink ? msp_osd_dp_cmd.attr | DISPLAYPORT_MSP_ATTR_BLINK : 0;	// Blink doesn't work with HDZero Freestyle V2 VTX
	memcpy(output, &msp_osd_dp_cmd, sizeof(msp_osd_dp_cmd));
	memcpy(&output[MSP_OSD_DP_WRITE_PAYLOAD], string, str_len);
	return 0;
}

// Construct a HDZero OSD draw command
displayportMspCommand_e construct_OSD_draw(){
	return MSP_DP_DRAW_SCREEN;
}

// Construct a HDZero OSD config command
msp_osd_dp_config_t construct_OSD_config(resolutionType_e resolution, uint8_t fontType){
	msp_osd_dp_config_t msp_osd_dp_config;
	msp_osd_dp_config.subcmd     = MSP_DP_CONFIG;
	msp_osd_dp_config.fontType   = fontType;
	msp_osd_dp_config.resolution = resolution;
	return msp_osd_dp_config;
}

void warn_to_upper(char* string){
	// Convert string to uppercase, otherwise it will try to print symbols instead
	for(size_t i=0; i<strlen(string);++i){
		string[i] = (string[i] >= 'a' && string[i] <= 'z') ? string[i] - 'a' + 'A' : string[i];
	}
}

} // namespace msp_dp_osd
