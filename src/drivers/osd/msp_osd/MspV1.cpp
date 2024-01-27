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

#include <px4_platform_common/px4_config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include "msp_defines.h"
#include "MspV1.hpp"

#include <px4_platform_common/log.h>

MspV1::MspV1(int fd) :
	_fd(fd)
{
}

int MspV1::GetMessageSize(int message_type)
{
	return 0;
}

struct msp_message_descriptor_t {
	uint8_t message_id;
	bool fixed_size;
	uint8_t message_size;
};

// #define MSP_DESCRIPTOR_COUNT 11
#define MSP_DESCRIPTOR_COUNT 16
const msp_message_descriptor_t msp_message_descriptors[MSP_DESCRIPTOR_COUNT] = {
	{MSP_OSD_CONFIG, true, sizeof(msp_osd_config_t)},
	{MSP_NAME, true, sizeof(msp_name_t)},
	{MSP_ANALOG, true, sizeof(msp_analog_t)},
	{MSP_STATUS, true, sizeof(msp_status_HDZ_t)},
	{MSP_BATTERY_STATE, true, sizeof(msp_battery_state_t)},
	{MSP_RAW_GPS, true, sizeof(msp_raw_gps_t)},
	{MSP_ATTITUDE, true, sizeof(msp_attitude_t)},
	{MSP_ALTITUDE, true, sizeof(msp_altitude_t)},
	{MSP_COMP_GPS, true, sizeof(msp_comp_gps_t)},
	{MSP_ESC_SENSOR_DATA, true, sizeof(msp_esc_sensor_data_dji_t)},
	{MSP_MOTOR_TELEMETRY, true, sizeof(msp_motor_telemetry_t)},
	{MSP_RC, true, sizeof(msp_rc_t)},
	{MSP_SET_OSD_CANVAS, true, sizeof(msp_osd_canvas_t)},
	{MSP_FC_VARIANT, true, sizeof(msp_fc_variant_t)},
	{MSP_VTX_CONFIG, true, sizeof(msp_VTX_config_t)},
	{MSP_CMD_DISPLAYPORT, false, sizeof(msp_osd_dp_cmd_t)},
};

#define MSP_FRAME_START_SIZE 5
#define MSP_CRC_SIZE 1
bool MspV1::Send(const uint8_t message_id, const void *payload)
{
	uint32_t payload_size = 0;

	msp_message_descriptor_t *desc = nullptr;

	for (int i = 0; i < MSP_DESCRIPTOR_COUNT; i++) {
		if (message_id == msp_message_descriptors[i].message_id) {
			desc = (msp_message_descriptor_t *)&msp_message_descriptors[i];
			break;
		}
	}
	
	if (!desc) {
		return false;
	}

	// need to handle different size Displayport commands
	if (!desc->fixed_size) {
		if (desc->message_id ==  MSP_CMD_DISPLAYPORT){
			uint8_t subcmd[1]{0};
			memcpy(subcmd, payload, 1);
			// PX4_INFO("DP SUBCMD: %u", subcmd[0]);
			if (subcmd[0] == MSP_DP_DRAW_SCREEN){
				payload_size = 1;
			} else if(subcmd[0] == MSP_DP_WRITE_STRING){	// Case when we write string.. payload size may vary 
				payload_size+=sizeof(msp_osd_dp_cmd_t);
				char dp_payload[sizeof(msp_osd_dp_cmd_t)+MSP_OSD_MAX_STRING_LENGTH];
				memcpy(dp_payload, payload, sizeof(dp_payload));
				// Find length of string in input (may not be whole array)
				for (int i=0;i<MSP_OSD_MAX_STRING_LENGTH;++i){
					if(dp_payload[MSP_OSD_DP_WRITE_PAYLOAD + i] == '\0') break;
					payload_size++;
				}
			} else {
				payload_size = desc->message_size;
			}
		} 
	} else {
		payload_size = desc->message_size;
	}

	uint8_t packet[MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE];
	uint8_t crc;

	packet[0] = '$';
	packet[1] = 'M';
	// packet[2] = '<';	// Need to find way to determine vtx type or pass this in maybe?
	packet[2] = '>';	// HDZero VTX firmware only supports 'replies'...
	packet[3] = payload_size;
	packet[4] = message_id;

	crc = payload_size ^ message_id;

	memcpy(packet + MSP_FRAME_START_SIZE, payload, payload_size);

	for (uint32_t i = 0; i < payload_size; i ++) {
		crc ^= packet[MSP_FRAME_START_SIZE + i];
	}

	packet[MSP_FRAME_START_SIZE + payload_size] = crc;

	int packet_size =  MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE;
	// Debug messages below
	// if (desc->message_id ==  MSP_CMD_DISPLAYPORT){
	// 	PX4_INFO("Payload size: %i",payload_size);
	// 	PX4_INFO("Packet size: %i",packet_size);
	// 	char ascii_string[packet_size * 3];
	// 	int ascii_string_index = 0;
	// 	for (int i = 0; i < packet_size; ++i) {
	// 		ascii_string_index += snprintf(ascii_string + ascii_string_index, sizeof(ascii_string) - ascii_string_index, "%02x ", packet[i]);
	// 	}
	// 	ascii_string[ascii_string_index - 1] = '\0';
	// 	PX4_INFO("[ %s ]", ascii_string);
	// }

	return  write(_fd, packet, packet_size) == packet_size;
}

#define MSP_FRAME_MSG_ID   4 
#define MSP_FRAME_MSG_SIZE 3 
// DO WE EVEN NEED TO READ IF WE ALREADY KNOW WHAT THE VTX WILL BE SENDING?? (IT'S THE SAME 4 CMDS OVER AND OVER FOR HDZERO FREESTYLE V2)
int MspV1::Read(){
	_msp_packet.index = 0;
	return read(_fd, _msp_packet.buffer, sizeof(_msp_packet.buffer));
}

int MspV1::processReadBuffer(int size){
	uint8_t i;
	uint8_t header_index = 0;
	uint8_t got_packet = 0;
	// PX4_INFO("Unprocessed buffer left: %i\tBuffer size: %i\tIndex: %u", size - _msp_packet.index, size, _msp_packet.index);
	if (_msp_packet.index >= size || size - _msp_packet.index < 6){
		// PX4_WARN("End of read buffer, size >= _msp_packet.index or Less than 6 bytes left.");
		_msp_packet.index = 0;
		return -1;
	}
	for (i=0; i < size - _msp_packet.index ; ++i){
		if(_msp_packet.buffer[_msp_packet.index + i] == MSP_HEADER){
			// Find location of header in read buffer
			got_packet = 1;
			header_index = i;

			// PX4_INFO("Found Packet header! Header index = %u", _msp_packet.index+header_index);
			// Accumulate ASCII values into a string
			// char ascii_string[18]; // Assuming each ASCII value is represented by 3 characters (e.g., "65 ")
			// int ascii_string_index = 0;
			// for (int j = 0; j < 18; ++j) {
				// ascii_string_index += snprintf(ascii_string + ascii_string_index, sizeof(ascii_string) - ascii_string_index, "%02x ", _msp_packet.buffer[_msp_packet.index + header_index + j]);
			// }
			// Remove the trailing space
			// ascii_string[ascii_string_index - 1] = '\0';
			// PX4_INFO("[ %s ]", ascii_string);
			break;
		}
	}

	// PX4_INFO("Current read buffer index = %u", _msp_packet.index+header_index);
	if (got_packet){
		// If we got a packet, extract msg id and payload size
		_msp_packet.message_id = _msp_packet.buffer[_msp_packet.index + header_index + MSP_FRAME_MSG_ID];
		_msp_packet.payload_size = _msp_packet.buffer[_msp_packet.index + header_index + MSP_FRAME_MSG_SIZE];
		if (_msp_packet.payload_size != 0){
			// PX4_WARN("Incorrect payload size: %u, resetting buffer index. Previous: %u", _msp_packet.payload_size,_msp_packet.index);
			_msp_packet.index = 0;
			return -1;
		}
		// Update our read buffer pointer
		_msp_packet.index += header_index + MSP_FRAME_START_SIZE + _msp_packet.payload_size + MSP_CRC_SIZE;
		// PX4_INFO("Message ID: %02x\tPayload Size: %02x\tNew Buffer Index: %i", _msp_packet.message_id, _msp_packet.payload_size, _msp_packet.index);
		return _msp_packet.message_id;
	} else {
		// PX4_WARN("Failed to find MSP packet, resetting buffer index. Previous: %u", _msp_packet.index);
		_msp_packet.index = 0;
		return -1;
	}
	return 0;
}

int MspV1::mspProcessCmds()
{

	int size = MspV1::Read();
	int res = 0;

	// Nothing to read
	if (size <= 0){
		return -1;
	}

	res = processReadBuffer(size);
	while(res >= 0 ){
		if (res < 0) {
			PX4_WARN("Read no packet");
			break;
		}
		// PX4_INFO("Processing message id: %i", res);
		switch (_msp_packet.message_id) {
		case MSP_FC_VARIANT:
			// PX4_WARN("FC Variant request");
			this->variant = 1;
			// PX4_INFO("Sending FC VARIANT\n");
			// const auto msg = msp_osd::construct_FC_VARIANT();
			// this->Send(MSP_FC_VARIANT, &msg);
			break;
		case MSP_STATUS:
			// PX4_WARN("STATUS request");
			this->status = 1;
			// PX4_INFO("Sending MSP STATUS\n");
			// vehicle_status_s vehicle_status{};
			// _vehicle_status_sub.copy(&vehicle_status);
			// const auto msg = msp_osd::construct_STATUS_HDZ(vehicle_status);
			// this->Send(MSP_STATUS, &msg);
			break;
		case MSP_RC:
			// PX4_WARN("RC request");
			this->rc = 1;
			// PX4_INFO("Sending RC\n");
			// input_rc_s input_rc{};
			// _input_rc_sub.copy(&input_rc);
			// const auto msg = msp_osd::construct_RC(input_rc);
			// this->Send(MSP_RC, &msg);
			break;
		case MSP_OSD_CANVAS:
			// PX4_WARN("OSD CANVAS request");
			this->osd_canvas = 1;
			// PX4_INFO("Sending OSD CANVAS\n");
			// const auto msg = msp_osd::construct_OSD_canvas();
			// this->Send(MSP_SET_OSD_CANVAS, &msg);
			break;    
		case MSP_VTX_CONFIG:
			// PX4_WARN("VTX CONFIG request");
			this->vtx_config = 1;
			// PX4_INFO("Sending VTX CONFIG\n");
			// const auto msg = msp_osd::construct_VTX_CONFIG();
			// this->Send(MSP_VTX_CONFIG, &msg);
			break;
		default: 
			PX4_WARN("UNKNOWN COMMAND: %02x\n", _msp_packet.message_id);
			this->unknown_cmd++;
			break;
		}
		res = processReadBuffer(size);
	}

	memset(&_msp_packet.buffer,0,sizeof(_msp_packet.buffer));

	return 0;
}