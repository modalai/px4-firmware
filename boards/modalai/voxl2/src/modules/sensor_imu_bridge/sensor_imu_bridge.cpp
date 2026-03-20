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
#include "mpa.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/sensor_accel.h>

class SensorImuBridge : public ModuleBase<SensorImuBridge>, public px4::WorkItem
{
public:

	SensorImuBridge();
	~SensorImuBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Subscribe to the TC-compensated accel topic (triggers the work item)
	uORB::SubscriptionCallbackWorkItem _vehicle_accel_sub{this, ORB_ID(vehicle_acceleration)};

	// Poll the TC-compensated gyro topic on each accel callback
	uORB::Subscription _vehicle_gyro_sub{ORB_ID(vehicle_angular_velocity)};

	// Poll a raw accel topic to get the temperature reading
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};

	int imu_pipe_ch{0};

};

SensorImuBridge::SensorImuBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool SensorImuBridge::init()
{
	if (MPA::Initialize() == -1) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char imu_pipe_name[] = "px4_sensor_imu";
	imu_pipe_ch = MPA::PipeCreate(imu_pipe_name);
	if (imu_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", imu_pipe_name);
		return false;
	}

	if (!_vehicle_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void SensorImuBridge::Run()
{
	if (should_exit()) {
		_vehicle_accel_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	vehicle_acceleration_s accel_data;

	if (_vehicle_accel_sub.update(&accel_data)) {

		imu_data_t imu;
		memset(&imu, 0, sizeof(imu));
		imu.magic_number = IMU_MAGIC_NUMBER;

		// TC-compensated acceleration (m/s^2)
		imu.accl_ms2[0] = accel_data.xyz[0];
		imu.accl_ms2[1] = accel_data.xyz[1];
		imu.accl_ms2[2] = accel_data.xyz[2];

		// TC-compensated angular velocity (rad/s)
		vehicle_angular_velocity_s gyro_data;
		if (_vehicle_gyro_sub.update(&gyro_data)) {
			imu.gyro_rad[0] = gyro_data.xyz[0];
			imu.gyro_rad[1] = gyro_data.xyz[1];
			imu.gyro_rad[2] = gyro_data.xyz[2];
		}

		// Get temperature from the raw accel sensor
		sensor_accel_s raw_accel;
		if (_sensor_accel_sub.update(&raw_accel)) {
			imu.temp_c = raw_accel.temperature;
		} else {
			imu.temp_c = IMU_INVALID_TEMPERATURE_VALUE;
		}

		imu.timestamp_ns = accel_data.timestamp * 1000; // Convert µs to ns

		if (MPA::PipeWrite(imu_pipe_ch, (void*)&imu, sizeof(imu_data_t)) == -1) {
			PX4_ERR("Pipe %d write failed!", imu_pipe_ch);
		}
	}
}

int SensorImuBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorImuBridge::task_spawn(int argc, char *argv[])
{
	SensorImuBridge *instance = new SensorImuBridge();

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

int SensorImuBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Sensor IMU bridge. Publishes TC-compensated accel and gyro data
from PX4 to the apps processor via MPA pipe.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_imu_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_imu_bridge_main(int argc, char *argv[])
{
	return SensorImuBridge::main(argc, argv);
}
