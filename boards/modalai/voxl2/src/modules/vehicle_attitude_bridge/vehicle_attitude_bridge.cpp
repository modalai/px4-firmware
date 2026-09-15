/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, inc. All rights reserved.
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

#include "mpa.hpp"

#include <pipe_interfaces/px4_vehicle_attitude_t.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/uORB.h>

static_assert(sizeof(px4_vehicle_attitude_t) == 64, "Unexpected attitude packet size");

class VehicleAttitudeBridge : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	VehicleAttitudeBridge();
	~VehicleAttitudeBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void request_stop() override;

private:
	void Run() override;
	void cleanup();

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	vehicle_attitude_s _vehicle_attitude{};

	int _pipe_ch{-1};
};

ModuleBase::Descriptor VehicleAttitudeBridge::desc{task_spawn, custom_command, print_usage};

VehicleAttitudeBridge::VehicleAttitudeBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VehicleAttitudeBridge::~VehicleAttitudeBridge()
{
	cleanup();
}

void VehicleAttitudeBridge::cleanup()
{
	_vehicle_attitude_sub.unregisterCallback();

	if (_pipe_ch >= 0) {
		MPA::PipeServerClose(_pipe_ch);
		_pipe_ch = -1;
	}
}

bool VehicleAttitudeBridge::init()
{
	if (MPA::Initialize() < 0) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char pipe_name[] = PX4_VEHICLE_ATTITUDE_PIPE_NAME;
	_pipe_ch = MPA::PipeCreate(pipe_name);

	if (_pipe_ch < 0) {
		PX4_ERR("Pipe create failed for %s", pipe_name);
		return false;
	}

	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		cleanup();
		return false;
	}

	// Also forward an existing attitude sample when starting the bridge.
	ScheduleNow();
	return true;
}

void VehicleAttitudeBridge::request_stop()
{
	ModuleBase::request_stop();
	ScheduleNow();
}

void VehicleAttitudeBridge::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		px4_vehicle_attitude_t attitude{};
		attitude.magic_number = PX4_VEHICLE_ATTITUDE_MAGIC_NUMBER;
		attitude.version = PX4_VEHICLE_ATTITUDE_VERSION;
		attitude.timestamp_ns = static_cast<int64_t>(_vehicle_attitude.timestamp) * 1000;
		attitude.timestamp_sample_ns = static_cast<int64_t>(_vehicle_attitude.timestamp_sample) * 1000;

		// Keep the attitude and reset metadata from the same uORB sample.
		for (int i = 0; i < 4; i++) {
			attitude.q[i] = _vehicle_attitude.q[i];
			attitude.delta_q_reset[i] = _vehicle_attitude.delta_q_reset[i];
		}

		attitude.quat_reset_counter = _vehicle_attitude.quat_reset_counter;

		if (MPA::PipeWrite(_pipe_ch, &attitude, sizeof(attitude)) < 0) {
			PX4_ERR("Pipe %d write failed", _pipe_ch);
		}
	}
}

int VehicleAttitudeBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleAttitudeBridge::task_spawn(int argc, char *argv[])
{
	VehicleAttitudeBridge *instance = new VehicleAttitudeBridge();

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int VehicleAttitudeBridge::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Vehicle attitude bridge - publishes vehicle_attitude to the px4_vehicle_attitude
MPA pipe as px4_vehicle_attitude_t, including attitude reset deltas and counters.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vehicle_attitude_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_attitude_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(VehicleAttitudeBridge::desc, argc, argv);
}
