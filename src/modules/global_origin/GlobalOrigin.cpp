/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "GlobalOrigin.hpp"

GlobalOrigin::GlobalOrigin() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

GlobalOrigin::~GlobalOrigin()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool GlobalOrigin::init()
{
	// execute Run() on every vehicle_status publication
	if (!_vehicle_status_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void GlobalOrigin::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (!_armed) {

	}

	//  update vehicle_status to check arming state
	if ((!_armed) && (_vehicle_status_sub.updated())) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	if ((!_armed) && (!_home_position_set)) {
		_vcmd.param5 = 47.397751;
		_vcmd.param6 = 8.545607;
		_vcmd.param7 = 488.10809;
		_vcmd.command = vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
		_vcmd.target_system = 1;
		_vcmd.target_component = 0;
		_vcmd.source_system = 1;
		_vcmd.source_component = 0;
		_vcmd.confirmation = false;
		_vcmd.from_external = true;
		_vcmd.timestamp = hrt_absolute_time();

		_cmd_pub.publish(_vcmd);

		_home_position_set = true;

		PX4_ERR("Published global origin from GlobalOrigin");
	}

	//if (!_armed) {
		_vpos.timestamp = hrt_absolute_time();
		_vpos.timestamp_sample = hrt_absolute_time() - 10;
		_vpos.lat = 47.397751;
		_vpos.lon = 8.545607;
		_vpos.alt = 488.10809;
		_vpos.alt_ellipsoid = 0.0;
		_vpos.delta_alt = 100.0;
		_vpos.lat_lon_reset_counter = 0;
		_vpos.alt_reset_counter = 0;
		_vpos.eph = 0.0;
		_vpos.epv = 0.0;
		_vpos.terrain_alt = 0.0;
		_vpos.terrain_alt_valid = false;
		_vpos.dead_reckoning = false;

		_pos_pub.publish(_vpos);

		//PX4_ERR("Published global position from GlobalOrigin");
	//}

	perf_end(_loop_perf);
}

int GlobalOrigin::task_spawn(int argc, char *argv[])
{
	GlobalOrigin *instance = new GlobalOrigin();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GlobalOrigin::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int GlobalOrigin::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GlobalOrigin::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int global_origin_main(int argc, char *argv[])
{
	return GlobalOrigin::main(argc, argv);
}
