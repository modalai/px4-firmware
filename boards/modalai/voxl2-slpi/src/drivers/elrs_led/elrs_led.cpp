/****************************************************************************
 *
 *   Copyright (c) 2023 ModalAI, Inc. All rights reserved.
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


#include <string>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Subscription.hpp>
#include <drivers/device/qurt/uart.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <lib/parameters/param.h>



extern "C" { __EXPORT int elrs_led_main(int argc, char *argv[]); }

namespace elrs_led
{

std::string _port = "7";
int _uart_fd = -1;
bool _initialized = false;
bool _is_running = false;

static px4_task_t _task_handle = -1;

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	if (_uart_fd < 0) {
		_uart_fd = qurt_uart_open(_port.c_str(), 921600);
	}

	if (_uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	}

	_initialized = true;

	return 0;
}

void elrs_led_task() {

	PX4_INFO("Starting task for elrs_led");

	int manual_control_input_fd  = orb_subscribe(ORB_ID(manual_control_input));

	px4_pollfd_struct_t fds[1] = { { .fd = manual_control_input_fd,  .events = POLLIN }	};	

	struct manual_control_setpoint_s setpoint_req;

	_is_running = true;

	while (true) {
		px4_poll(fds, 1, 10000);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(manual_control_input), manual_control_input_fd, &setpoint_req);

			PX4_INFO("Got aux1 0x%x", (uint32_t) setpoint_req.aux1);
		} else {
			PX4_INFO("Poll failed");
		}
	}
}

int start(int argc, char *argv[]) {

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			_port = myoptarg;
			PX4_INFO("Setting port to %s", _port.c_str());
			break;
		default:
			break;
		}
	}

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	if (_is_running) {
		PX4_WARN("Already started");
		return 0;
	}

	_task_handle = px4_task_spawn_cmd("elrs_led_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t) &elrs_led_task,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: elrs_led start [options]");
	PX4_INFO("Options: -p <number>    uart port number");
}

} // End namespance elrs_led

int elrs_led_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		elrs_led::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return elrs_led::start(argc - 1, argv + 1);
	} else {
		elrs_led::usage();
		return -1;
	}

	return 0;
}
