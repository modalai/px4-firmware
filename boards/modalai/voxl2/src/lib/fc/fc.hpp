/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, inc. All rights reserved.
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
#include <fc_sensor.h>

#pragma once

class FC {
public:
	static int Initialize(bool enable_debug_messages, fc_callbacks *callbacks);

	static uint64_t GetDspTimestampUs(void);
	static int GetTimeOffset(void);
	static int SetLibraryName(const char *name);
	static int Advertise(const char *topic);
	static int Subscribe(const char *topic);
	static int Unsubscribe(const char *topic);
	static int SendData(const char *topic,
						const uint8_t *data,
						uint32_t length_in_bytes);
	static void KillSlpi(void);

private:

	typedef uint64_t (*fc_sensor_get_dsp_timestamp_us_t)(void);
	typedef int (*fc_sensor_get_time_offset_t)(void);
	typedef int (*fc_sensor_set_library_name_t)(const char *name);
	typedef int (*fc_sensor_initialize_t)(bool enable_debug_messages, fc_callbacks *callbacks);
	typedef int (*fc_sensor_advertise_t)(const char *topic);
	typedef int (*fc_sensor_subscribe_t)(const char *topic);
	typedef int (*fc_sensor_unsubscribe_t)(const char *topic);
	typedef int (*fc_sensor_send_data_t)(const char *topic,
										const uint8_t *data,
										uint32_t length_in_bytes);
	typedef void (*fc_sensor_kill_slpi_t)(void);

	static fc_sensor_get_dsp_timestamp_us_t get_dsp_timestamp_us;
	static fc_sensor_get_time_offset_t get_time_offset;
	static fc_sensor_set_library_name_t set_library_name;
	static fc_sensor_initialize_t initialize;
	static fc_sensor_advertise_t advertise;
	static fc_sensor_subscribe_t subscribe;
	static fc_sensor_unsubscribe_t unsubscribe;
	static fc_sensor_send_data_t send_data;
	static fc_sensor_kill_slpi_t kill_slpi;

	static bool initialized;
	static void *handle;

	static int current_client;
	static int current_server;

	static const int MAX_MPA_CLIENTS{8};
	static mpa_data_cb_t data_cb[MAX_MPA_CLIENTS];
};
