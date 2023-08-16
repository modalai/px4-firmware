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


#include <string>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/modal_io_data.h>
#include <lib/parameters/param.h>


extern "C" { __EXPORT int modal_io_bridge_main(int argc, char *argv[]); }

namespace modal_io_bridge
{

bool _initialized = false;
bool _is_running = false;

static px4_task_t _task_handle = -1;

uORB::Publication<modal_io_data_s> _data_pub{ORB_ID(modal_io_data)};

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	_initialized = true;

	return 0;
}

void modal_io_bridge_task() {
	
	modal_io_data_s	io_data;

	_is_running = true;

	while (true) {

		usleep(20000); // Update every 20ms

		memset(&io_data, 0, sizeof(modal_io_data_s));

		io_data.timestamp = hrt_absolute_time();

		_data_pub.publish(io_data);

	}
}

int start(int argc, char *argv[]) {

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	if (_is_running) {
		PX4_WARN("Already started");
		return 0;
	}

	_task_handle = px4_task_spawn_cmd("modal_io_bridge_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t) &modal_io_bridge_task,
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
	PX4_INFO("Usage: modal_io_bridge start");
}

} // End namespance modal_io_bridge

int modal_io_bridge_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		modal_io_bridge::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return modal_io_bridge::start(argc - 1, argv + 1);
	} else {
		modal_io_bridge::usage();
		return -1;
	}

	return 0;
}
