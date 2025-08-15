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

#include <modal_pipe.h>
#include <dlfcn.h>
#include <px4_log.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/module.h>
// #include <uORB/uORB.h>
// #include <uORB/SubscriptionCallback.hpp>
// #include <uORB/topics/modal_io_mavlink_data.h>


#include <string.h>
// #include <px4_log.h>
// #include <px4_platform_common/tasks.h>
// #include <px4_platform_common/getopt.h>
// #include <uORB/PublicationMulti.hpp>
// #include <drivers/drv_hrt.h>
// #include <uORB/topics/input_rc.h>
// #include <lib/parameters/param.h>
// #include <poll.h>
// #include <termios.h>

extern "C" { __EXPORT int vfc_status_logger_main(int argc, char *argv[]); }

namespace vfc_status_logger
{

bool _initialized = false;
// bool _is_running = false;

// static px4_task_t _task_handle = -1;

// uORB::PublicationMulti<input_rc_s> _rc_pub{ORB_ID(input_rc)};

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

// void vfc_status_logger_task() {
// 
// 	_is_running = true;
// 
// 	while (true) {
// 
// 		usleep(20000); // Update every 20ms
// 
// 		// _rc_pub.publish(rc_val);
// 	}
// }

int start(int argc, char *argv[]) {

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	// if (_is_running) {
	// 	PX4_WARN("Already started");
	// 	return 0;
	// }

	// _task_handle = px4_task_spawn_cmd("vfc_status_logger_main",
	// 				  SCHED_DEFAULT,
	// 				  SCHED_PRIORITY_DEFAULT,
	// 				  2000,
	// 				  (px4_main_t) &vfc_status_logger_task,
	// 				  (char *const *)argv);

	// if (_task_handle < 0) {
	// 	PX4_ERR("task start failed");
	// 	return -1;
	// }

	PX4_INFO("vfc_status_logger starting");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: vfc_status_logger start");
}

} // End namespance vfc_status_logger

int vfc_status_logger_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		vfc_status_logger::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return vfc_status_logger::start(argc - 1, argv + 1);
	} else {
		vfc_status_logger::usage();
		return -1;
	}

	return 0;
}