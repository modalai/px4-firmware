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

#include "fc.hpp"
#include <dlfcn.h>
#include <px4_log.h>
#include <string.h>

bool FC::initialized = false;
void *FC::handle = nullptr;

FC::fc_sensor_get_dsp_timestamp_us_t FC::fc_sensor_get_dsp_timestamp_us = nullptr;
FC::fc_sensor_get_time_offset_t FC::fc_sensor_get_time_offset = nullptr;
FC::fc_sensor_set_library_name_t FC::fc_sensor_set_library_name = nullptr;
FC::fc_sensor_initialize_t FC::fc_sensor_initialize = nullptr;
FC::fc_sensor_advertise_t FC::fc_sensor_advertise = nullptr;
FC::fc_sensor_subscribe_t FC::fc_sensor_subscribe = nullptr;
FC::fc_sensor_unsubscribe_t FC::fc_sensor_unsubscribe = nullptr;
FC::fc_sensor_send_data_t FC::fc_sensor_send_data = nullptr;
FC::fc_sensor_kill_slpi_t FC::fc_sensor_kill_slpi = nullptr;

uint64_t FC::GetDspTimestampUs(void) {
	return get_dsp_timestamp_us();
}

int FC::GetTimeOffset(void) {
	return get_time_offset(void);
}

int FC::SetLibraryName(const char *name) {
	return set_library_name(const char *name);
}

int FC::Advertise(const char *topic) {
	return advertise(topic);
}

int FC::Subscribe(const char *topic) {
	return subscribe(topic);
}

int FC::Unsubscribe(const char *topic) {
	return unsubscribe(topic);
}

int FC::SendData(const char *topic, const uint8_t *data, uint32_t length_in_bytes) {
	return send_data(topic, data, length_in_bytes);
}

void FC::KillSlpi(void) {
	kill_slpi();
}


int FC::Initialize()
{
	if (initialized) {
		// Already successfully initialized
		return 0;
	}

	char libname[] = "libfc_sensor.so";
	handle = dlopen(libname, RTLD_LAZY | RTLD_GLOBAL);
	if (!handle) {
		PX4_ERR("Error opening library %s: %s\n", libname, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded library %s", libname);
	}

	char open_pipe_name[] = "pipe_client_open";
	open_pipe = (pipe_client_open_t) dlsym(handle, open_pipe_name);
	if (!open_pipe) {
		PX4_ERR("Error finding symbol %s: %s", open_pipe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", open_pipe_name);
	}

	char get_dsp_timestamp_us_name[] = "fc_sensor_get_dsp_timestamp_us";
	get_dsp_timestamp_us = (get_dsp_timestamp_us_t) dlsym(handle, get_dsp_timestamp_us_name);
	if (!get_dsp_timestamp_us) {
		PX4_ERR("Error finding symbol %s: %s", get_dsp_timestamp_us_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", get_dsp_timestamp_us_name);
	}

	char get_time_offset_name[] = "fc_sensor_get_time_offset";
	get_time_offset = (get_time_offset_t) dlsym(handle, get_time_offset_name);
	if (!get_time_offset) {
		PX4_ERR("Error finding symbol %s: %s", get_time_offset_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", get_time_offset_name);
	}

	char set_library_name_name[] = "fc_sensor_set_library_name";
	set_library_name = (set_library_name_t) dlsym(handle, set_library_name_name);
	if (!set_library_name) {
		PX4_ERR("Error finding symbol %s: %s", set_library_name_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", set_library_name_name);
	}

	char initialize_name[] = "fc_sensor_initialize";
	initialize = (initialize_t) dlsym(handle, initialize_name);
	if (!initialize) {
		PX4_ERR("Error finding symbol %s: %s", initialize_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", initialize_name);
	}

	char advertise_name[] = "fc_sensor_advertise";
	advertise = (advertise_t) dlsym(handle, advertise_name);
	if (!advertise) {
		PX4_ERR("Error finding symbol %s: %s", advertise_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", advertise_name);
	}

	char subscribe_name[] = "fc_sensor_subscribe";
	subscribe = (subscribe_t) dlsym(handle, subscribe_name);
	if (!subscribe) {
		PX4_ERR("Error finding symbol %s: %s", subscribe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", subscribe_name);
	}

	char unsubscribe_name[] = "fc_sensor_unsubscribe";
	unsubscribe = (unsubscribe_t) dlsym(handle, unsubscribe_name);
	if (!unsubscribe) {
		PX4_ERR("Error finding symbol %s: %s", unsubscribe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", unsubscribe_name);
	}

	char send_data_name[] = "fc_sensor_send_data";
	send_data = (send_data_t) dlsym(handle, send_data_name);
	if (!send_data) {
		PX4_ERR("Error finding symbol %s: %s", send_data_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", send_data_name);
	}

	char kill_slpi_name[] = "fc_sensor_kill_slpi";
	kill_slpi = (kill_slpi_t) dlsym(handle, kill_slpi_name);
	if (!kill_slpi) {
		PX4_ERR("Error finding symbol %s: %s", kill_slpi_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", kill_slpi_name);
	}

	initialized = true;

	return 0;
}

