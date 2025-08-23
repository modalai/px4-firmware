/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, Inc. All rights reserved.
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

#include <modal_pipe.h>
#include <dlfcn.h>
#include <px4_log.h>
#include <string.h>

namespace mpa
{

static bool _initialized = false;

// called whenever we connect or reconnect to the server
static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	PX4_INFO("vfc status server connected");
	return;
}

// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	PX4_INFO("vfc status server disconnected");
	return;
}

static void _helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// PX4_INFO("Got %d bytes in pipe callback", bytes);

	int packet_size = sizeof(vfc_data_t);
	// validate that the data makes sense
	int n_packets = 0;
	if ((! bytes) || (bytes % packet_size)) {
		PX4_ERR("Invalid data size: %d", bytes);
	} else {
		n_packets = bytes / packet_size;
		for (int i = 0; i < n_packets; i++) {
			// _print_data((offboard_log_packet) *((offboard_log_packet*) (data + (packet_size * i))));
		}
	}

	return;
}

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	char libname[] = "libmodal_pipe.so";
	void *handle = dlopen(libname, RTLD_LAZY | RTLD_GLOBAL);
	if (!handle) {
		PX4_ERR("Error opening library %s: %s\n", libname, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded library %s", libname);
	}

	// set up all our MPA callbacks
	typedef int (*pipe_client_set_simple_helper_cb_t)(int ch, client_simple_cb* cb, void* context);
	char func1_name[] = "pipe_client_set_simple_helper_cb";
	pipe_client_set_simple_helper_cb_t func1_ptr = (pipe_client_set_simple_helper_cb_t) dlsym(handle, func1_name);
	if (!func1_ptr) {
		PX4_ERR("Error finding symbol %s: %s\n", func1_name, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded function %s", func1_name);
	}
	func1_ptr(0, _helper_cb, NULL);

	typedef int (*pipe_client_set_connect_cb_t)(int ch, client_connect_cb* cb, void* context);
	char func2_name[] = "pipe_client_set_connect_cb";
	pipe_client_set_connect_cb_t func2_ptr = (pipe_client_set_connect_cb_t) dlsym(handle, func2_name);
	if (!func2_ptr) {
		PX4_ERR("Error finding symbol %s: %s", func2_name, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded function %s", func2_name);
	}
	func2_ptr(0, _connect_cb, NULL);

	typedef int (*pipe_client_set_disconnect_cb_t)(int ch, client_disc_cb* cb, void* context);
	char func3_name[] = "pipe_client_set_disconnect_cb";
	pipe_client_set_disconnect_cb_t func3_ptr = (pipe_client_set_disconnect_cb_t) dlsym(handle, func3_name);
	if (!func3_ptr) {
		PX4_ERR("Error finding symbol %s: %s", func3_name, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded function %s", func3_name);
	}
	func3_ptr(0, _disconnect_cb, NULL);

	// request a new pipe from the server
	typedef int (*pipe_client_open_t)(int ch, const char* name_or_location, const char* client_name, int flags, int buf_len);
	char func4_name[] = "pipe_client_open";
	pipe_client_open_t func4_ptr = (pipe_client_open_t) dlsym(handle, func4_name);
	if (!func4_ptr) {
		PX4_ERR("Error finding symbol %s: %s", func4_name, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded function %s", func4_name);
	}
	printf("waiting for server\n");
	char vfc_pipe_name[] = "vfc";
	if (func4_ptr(0, vfc_pipe_name, "px4", EN_PIPE_CLIENT_SIMPLE_HELPER, sizeof(vfc_data_t) * 10) < 0) {
		PX4_ERR("Error opening pipe %s", vfc_pipe_name);
		return -1;
	}

	_initialized = true;

	return 0;
}

} // End namespance mpa

