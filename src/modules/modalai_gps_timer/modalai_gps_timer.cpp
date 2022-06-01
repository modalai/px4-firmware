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

#include "modalai_gps_timer.hpp"

ModalaiGPSTimer::ModalaiGPSTimer() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "modalai_gps_timer control")) // TODO : do we even need these perf counters
{
}

ModalaiGPSTimer::~ModalaiGPSTimer()
{
	perf_free(_loop_perf);
}

int ModalaiGPSTimer::custom_command(int argc, char *argv[])
{
	int ret = ModalaiGPSTimer::task_spawn(argc, argv);

	if (ret) {
		return ret;
	}
	return print_usage("unknown command");
}

int ModalaiGPSTimer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module resets the system time from the GPS provided there is a large diff between the two.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("modalai_gps_timer", "start");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

ModalaiGPSTimer *ModalaiGPSTimer::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	ModalaiGPSTimer *instance = new ModalaiGPSTimer();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate ModalaiGPSTimer object");
	}

	return instance;
}

int ModalaiGPSTimer::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("modalai_gps_timer",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1700,
				      (px4_main_t)&ModalaiGPSTimer::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void ModalaiGPSTimer::run()
{
	/* Get the latest GPS publication */
	vehicle_gps_position_s gps_pos;
	time_t utc_time_sec;

	while (true){

		time_t system_time = time (NULL);

		PX4_DEBUG("SYSTEM_TIME: %lf", (double) system_time);

		if (vehicle_gps_position_sub.copy(&gps_pos)) {
			utc_time_sec = gps_pos.time_utc_usec / 1e6;
			PX4_DEBUG("GPS_TIME: %lf", (double) utc_time_sec);

			if (gps_pos.fix_type >= 2 && utc_time_sec >= GPS_EPOCH_SECS) {
				if(utc_time_sec - system_time >= 60){
					PX4_INFO("seting UTC clock from GPS to: %s", asctime(gmtime(&utc_time_sec)));
					if(stime(&utc_time_sec)<0){
						PX4_ERR("Failed to set system time to GPS time");
						return;
					}
					break;
				} else {
					PX4_INFO("System time only 1 minute from GPS time, keeping system time from cache.");
					break;
				}
			} else {
				PX4_DEBUG("GPS fix not above 2 yet.");
			}
		}
		usleep(5000000);
	}
}

extern "C" __EXPORT int modalai_gps_timer_main(int argc, char *argv[])
{
	return ModalaiGPSTimer::main(argc, argv);
}
