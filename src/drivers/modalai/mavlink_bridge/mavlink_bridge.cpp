/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
#include <stdint.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <v2.0/standard/mavlink.h>
#include "uORB/uORBManager.hpp"
#include <uORB/topics/mavlink_msg.h>

#include <px4_log.h>

extern "C" { __EXPORT int mavlink_bridge_main(int argc, char *argv[]); }

namespace mavlink_bridge
{

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
bool debug = false;

int start(int argc, char *argv[]);
int stop();
int status();
void usage();

void task_main(int argc, char *argv[]) {
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			debug = true;
			PX4_INFO("Setting debug flag on");
			break;
		default:
			break;
		}
	}

	_is_running = true;

    int mavlink_rx_msg_fd = orb_subscribe(ORB_ID(mavlink_rx_msg));

	struct mavlink_msg_s incoming_msg;

    px4_pollfd_struct_t fds[1] = { { .fd = mavlink_rx_msg_fd,  .events = POLLIN } };

	while ( ! _task_should_exit) {
    	px4_poll(fds, 2, 1000);
    	if (fds[0].revents & POLLIN) {
    		orb_copy(ORB_ID(mavlink_rx_msg), mavlink_rx_msg_fd, &incoming_msg);
			PX4_INFO("Got incoming mavlink msg of length %u", incoming_msg.msg_len);
		}
		usleep(500);
	}
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("mavlink_bridge_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

int status() {
	PX4_INFO("running: %s", _is_running ? "yes" : "no");
	return 0;
}

void usage() {
	PX4_INFO("Usage: mavlink_bridge {start|info|stop}");
}

}

int mavlink_bridge_main(int argc, char *argv[]) {
	int myoptind = 1;

	if (argc <= 1) {
		mavlink_bridge::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return mavlink_bridge::start(argc - 1, argv + 1);
	} else if (!strcmp(verb, "stop")) {
		return mavlink_bridge::stop();
	} else if (!strcmp(verb, "status")) {
		return mavlink_bridge::status();
	} else {
		mavlink_bridge::usage();
		return -1;
	}
}
