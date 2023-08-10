/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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


#include <px4_log.h>
#include <drivers/device/qurt/uart.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/input_rc.h>
#include <lib/parameters/param.h>

#include "protocol.h"

#define ASYNC_UART_READ_WAIT_US 2000

int dsp_sbus_uart_fd = -1;
IOPacket dsp_sbus_packet;
uint64_t dsp_sbus_rc_last_valid;		///< last valid timestamp

extern "C" { __EXPORT int dsp_sbus_main(int argc, char *argv[]); }

int dsp_sbus_bus_exchange(IOPacket *_packet)
{
	int ret = 0;
	int read_retries = 3;
	int read_succeeded = 0;
	int packet_size = sizeof(IOPacket);

	(void) qurt_uart_write(dsp_sbus_uart_fd, (const char*) _packet, packet_size);

	usleep(100);

    // The UART read on SLPI is via an asynchronous service so specify a timeout
    // for the return. The driver will poll periodically until the read comes in
    // so this may block for a while. However, it will timeout if no read comes in.
	while (read_retries) {
    	ret = qurt_uart_read(dsp_sbus_uart_fd, (char*) _packet, packet_size, ASYNC_UART_READ_WAIT_US);
		if (ret) {
			PX4_INFO("Read %d bytes", ret);

			/* Check CRC */
			uint8_t crc = _packet->crc;
			_packet->crc = 0;

			if (crc != crc_packet(_packet)) {
				PX4_ERR("PX4IO packet CRC error");
				return -EIO;
			} else if (PKT_CODE(*_packet) == PKT_CODE_CORRUPT) {
				PX4_ERR("PX4IO packet corruption");
				return -EIO;
			} else {
				read_succeeded = 1;
				break;
			}
		}
		PX4_ERR("Read attempt failed");
		read_retries--;
	}

	if ( ! read_succeeded) {
		return -EIO;
	}

	return 0;
}

int dsp_sbus_io_reg_set(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	// if (num_values > ((_max_transfer) / sizeof(*values))) {
	// 	PX4_ERR("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
	// 	return -1;
	// }

	// int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);
	int ret = 0;

	dsp_sbus_packet.count_code = num_values | PKT_CODE_WRITE;
	dsp_sbus_packet.page = page;
	dsp_sbus_packet.offset = offset;
	memcpy((void*) &dsp_sbus_packet.regs[0], (void*) values, (2 * num_values));

	for (unsigned i = num_values; i < PKT_MAX_REGS; i++) {
		dsp_sbus_packet.regs[i] = 0x55aa;
	}

	dsp_sbus_packet.crc = 0;
	dsp_sbus_packet.crc = crc_packet(&dsp_sbus_packet);

	ret = dsp_sbus_bus_exchange(&dsp_sbus_packet);

	if (ret != 0) {
		PX4_ERR("px4io io_reg_set(%hhu,%hhu,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	return ret;
}

uint32_t dsp_sbus_io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return dsp_sbus_io_reg_set(page, offset, &value, 1);
}

int dsp_sbus_io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	// if (num_values > ((_max_transfer) / sizeof(*values))) {
	// 	PX4_ERR("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
	// 	return -1;
	// }

	// int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);
	int ret = 0;

	dsp_sbus_packet.count_code = num_values | PKT_CODE_READ;
	dsp_sbus_packet.page = page;
	dsp_sbus_packet.offset = offset;

	dsp_sbus_packet.crc = 0;
	dsp_sbus_packet.crc = crc_packet(&dsp_sbus_packet);

	ret = dsp_sbus_bus_exchange(&dsp_sbus_packet);

	if (ret != 0) {
		PX4_ERR("px4io io_reg_get(%hhu,%hhu,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	memcpy(values, &dsp_sbus_packet.regs[0], num_values * 2);

	return OK;
}

uint32_t dsp_sbus_io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (dsp_sbus_io_reg_get(page, offset, &value, 1) != OK) {
		// Registers are only 16 bit so any value over 0xFFFF can signal a fault
		return 0xFFFFFFFF;
	}

	return value;
}

int dsp_sbus_io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	uint16_t value = 0;
	int ret = dsp_sbus_io_reg_get(page, offset, &value, 1);

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits;
	value |= setbits;

	return dsp_sbus_io_reg_set(page, offset, value);
}

int dsp_sbus_detect()
{
	if (dsp_sbus_uart_fd < 0) {
		// dsp_sbus_uart_fd = qurt_uart_open("2", 1000000);
		// dsp_sbus_uart_fd = qurt_uart_open("7", 921600);
		dsp_sbus_uart_fd = qurt_uart_open("2", 921600);
	}

	if (dsp_sbus_uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	} else {
		PX4_INFO("dsp_sbus serial port fd %d", dsp_sbus_uart_fd);
	}

	/* get some parameters */
	unsigned protocol = dsp_sbus_io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

	if (protocol != 4) {
		PX4_ERR("dsp_sbus version error: %u", protocol);
		return -1;
	}

	PX4_INFO("dsp_sbus found");

	return 0;
}

int dsp_sbus_io_set_rc_config()
{
	unsigned offset = 0;
	int input_map[input_rc_s::RC_INPUT_MAX_CHANNELS];
	int32_t ichan;
	int ret = OK;

	/*
	 * Generate the input channel -> control channel mapping table;
	 * assign RC_MAP_ROLL/PITCH/YAW/THROTTLE to the canonical
	 * controls.
	 */

	/* fill the mapping with an error condition triggering value */
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		input_map[i] = UINT8_MAX;
	}

	/*
	 * NOTE: The indices for mapped channels are 1-based
	 *       for compatibility reasons with existing
	 *       autopilots / GCS'.
	 */

	/* ROLL */
	param_get(param_find("RC_MAP_ROLL"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 0;
	}

	/* PITCH */
	param_get(param_find("RC_MAP_PITCH"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 1;
	}

	/* YAW */
	param_get(param_find("RC_MAP_YAW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 2;
	}

	/* THROTTLE */
	param_get(param_find("RC_MAP_THROTTLE"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 3;
	}

	/* FLAPS */
	param_get(param_find("RC_MAP_FLAPS"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 4;
	}

	/* AUX 1*/
	param_get(param_find("RC_MAP_AUX1"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 5;
	}

	/* AUX 2*/
	param_get(param_find("RC_MAP_AUX2"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 6;
	}

	/* AUX 3*/
	param_get(param_find("RC_MAP_AUX3"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		input_map[ichan - 1] = 7;
	}

	/* MAIN MODE SWITCH */
	param_get(param_find("RC_MAP_MODE_SW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		/* use out of normal bounds index to indicate special channel */
		input_map[ichan - 1] = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
	}

	/*
	 * Iterate all possible RC inputs.
	 */
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;

		/*
		 * RC params are floats, but do only
		 * contain integer values. Do not scale
		 * or cast them, let the auto-typeconversion
		 * do its job here.
		 * Channels: 500 - 2500
		 * Inverted flag: -1 (inverted) or 1 (normal)
		 */

		sprintf(pname, "RC%u_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = fval;

		sprintf(pname, "RC%u_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = fval;

		sprintf(pname, "RC%u_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = fval;

		sprintf(pname, "RC%u_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = fval;

		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];

		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%u_REV", i + 1);
		param_get(param_find(pname), &fval);

		/*
		 * This has been taken for the sake of compatibility
		 * with APM's setup / mission planner: normal: 1,
		 * inverted: -1
		 */
		if (fval < 0) {
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
		}

		PX4_INFO("RC %u config: 0x%.4x 0x%.4x 0x%.4x 0x%.4x 0x%.4x 0x%.4x",
				 i + 1, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);

		/* send channel config to IO */
		ret = dsp_sbus_io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);

		if (ret != OK) {
			PX4_ERR("rc config upload failed");
			break;
		}

		/* check the IO initialisation flag */
		if (!(dsp_sbus_io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS) & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
			PX4_ERR("config for RC%u rejected by IO", i + 1);
			break;
		}

		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int dsp_sbus_configure() {
	
	dsp_sbus_io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);
	
	// uint16_t status_config = PX4IO_P_STATUS_FLAGS_RC_OK | PX4IO_P_STATUS_FLAGS_RC_SBUS | PX4IO_P_STATUS_FLAGS_ARM_SYNC | PX4IO_P_STATUS_FLAGS_INIT_OK;
	// status_config |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
	// 
	// dsp_sbus_io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, status_config);
	// 
	// dsp_sbus_io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
	// 
	// dsp_sbus_io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, 0x0000);

	// dsp_sbus_io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
	// 
	// 
	// /* dis-arm IO before touching anything */
	// dsp_sbus_io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
	// 	      PX4IO_P_SETUP_ARMING_FMU_ARMED |
	// 	      PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK |
	// 	      PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK |
	// 	      PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE |
	// 	      PX4IO_P_SETUP_ARMING_LOCKDOWN, 0);

	(void) dsp_sbus_io_set_rc_config();

	return 0;
}

int dsp_sbus_start() {
	
	uint16_t status_regs[2] {};
	if (dsp_sbus_io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &status_regs[0],
							sizeof(status_regs) / sizeof(status_regs[0])) == OK) {
		PX4_INFO("dsp_sbus status 0x%.4x", status_regs[0]);
		PX4_INFO("dsp_sbus alarms 0x%.4x", status_regs[1]);
	} else {
		PX4_ERR("Failed to read status / alarm registers");
	}

	/* fetch values from IO */
	if (!(status_regs[0] & PX4IO_P_STATUS_FLAGS_RC_OK)) {
		PX4_INFO("RC lost status flag set");
	} else {
		PX4_INFO("RC lost status flag is not set");
	}

	input_rc_s	rc_val;

	if (status_regs[0] & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;
		PX4_INFO("Got valid SBUS");
	} else {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
		PX4_INFO("SBUS not valid");
	}

	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t rc_regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	if (dsp_sbus_io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &rc_regs[0],
							sizeof(rc_regs) / sizeof(rc_regs[0])) != OK) {
		PX4_ERR("Failed to read RC registers");
	} else {
		PX4_INFO("Successfully read RC registers");
		PX4_INFO("Prolog: 0x%.4x 0x%.4x 0x%.4x 0x%.4x 0x%.4x 0x%.4x",
				 rc_regs[0], rc_regs[1], rc_regs[2], rc_regs[3], rc_regs[4], rc_regs[5]);
	}

	uint32_t channel_count = rc_regs[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		PX4_INFO("Got %u for channel count. Limiting to 18", channel_count);
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	rc_val.channel_count = channel_count;
	PX4_INFO("RC channel count: %u", rc_val.channel_count);

	rc_val.timestamp = hrt_absolute_time();
	rc_val.rc_ppm_frame_length = rc_regs[PX4IO_P_RAW_RC_DATA];
	
	// if (!_analog_rc_rssi_stable) {
	// 	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	// 
	// } else {
	// 	float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;
	// 
	// 	if (rssi_analog > 100.0f) {
	// 		rssi_analog = 100.0f;
	// 	}
	// 
	// 	if (rssi_analog < 0.0f) {
	// 		rssi_analog = 0.0f;
	// 	}
	// 
	// 	input_rc.rssi = rssi_analog;
	// }
	
	rc_val.rc_failsafe = (rc_regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	rc_val.rc_lost = !(rc_regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	rc_val.rc_lost_frame_count = rc_regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	rc_val.rc_total_frame_count = rc_regs[PX4IO_P_RAW_FRAME_COUNT];
	
	if (!rc_val.rc_lost && !rc_val.rc_failsafe) {
		dsp_sbus_rc_last_valid = rc_val.timestamp;
	}
	
	rc_val.timestamp_last_signal = dsp_sbus_rc_last_valid;

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		rc_val.values[i] = rc_regs[prolog + i];
		PX4_INFO("RC channel %u: 0x%.4x", i, rc_val.values[i]);
	}
	
	/* zero the remaining fields */
	for (unsigned i = channel_count; i < (sizeof(rc_val.values) / sizeof(rc_val.values[0])); i++) {
		rc_val.values[i] = 0;
	}
	
	// /* get RSSI from input channel */
	// if (_rssi_pwm_chan > 0 && _rssi_pwm_chan <= input_rc_s::RC_INPUT_MAX_CHANNELS && _rssi_pwm_max - _rssi_pwm_min != 0) {
	// 	int rssi = ((input_rc.values[_rssi_pwm_chan - 1] - _rssi_pwm_min) * 100) /
	// 		   (_rssi_pwm_max - _rssi_pwm_min);
	// 	rssi = rssi > 100 ? 100 : rssi;
	// 	rssi = rssi < 0 ? 0 : rssi;
	// 	input_rc.rssi = rssi;
	// }

	return 0;
}

int dsp_sbus_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return dsp_sbus_start();

	} else if (!strcmp(verb, "detect")) {
		return dsp_sbus_detect();

	} else if (!strcmp(verb, "configure")) {
		return dsp_sbus_configure();

	} else {
		return -1;
	}

	return 0;
}
