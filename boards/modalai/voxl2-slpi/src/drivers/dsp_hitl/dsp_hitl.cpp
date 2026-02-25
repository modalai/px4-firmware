/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

/**
 * @file dsp_hitl.cpp
 *
 * DSP-side HITL driver for all sensors (IMU, MAG, BARO).
 * Subscribes to hil_sensor_imu and hil_sensor_mag_baro (from apps proc via MUORB)
 * and republishes as sensor_accel/sensor_gyro/sensor_mag/sensor_baro with fresh DSP timestamps.
 */

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_log.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hil_sensor_imu.h>
#include <uORB/topics/hil_sensor_mag_baro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_status.h>

#include <unistd.h>

extern "C" { __EXPORT int dsp_hitl_main(int argc, char *argv[]); }

namespace dsp_hitl
{

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;

// Direct uORB publications - bypass PX4Accelerometer/PX4Gyroscope to have full timestamp control
uORB::Publication<sensor_accel_s> _sensor_accel_pub{ORB_ID(sensor_accel)};
uORB::Publication<sensor_gyro_s> _sensor_gyro_pub{ORB_ID(sensor_gyro)};
uORB::Publication<sensor_mag_s> _sensor_mag_pub{ORB_ID(sensor_mag)};
uORB::Publication<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

bool got_first_imu_msg = false;
bool got_first_mag_baro_msg = false;

// IMU data
float x_accel = 0;
float y_accel = 0;
float z_accel = 0;
float x_gyro = 0;
float y_gyro = 0;
float z_gyro = 0;

// MAG/BARO data
float x_mag = 0;
float y_mag = 0;
float z_mag = 0;
float pressure_pa = 0;
float temperature = 0;

// Status counters
uint32_t hil_imu_counter = 0;
uint32_t hil_mag_baro_counter = 0;
uint32_t imu_counter = 0;
uint32_t mag_counter = 0;
uint32_t baro_counter = 0;

vehicle_status_s _vehicle_status{};

int start(int argc, char *argv[]);
int stop();
int get_status();
void clear_status_counters();
void usage();
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	uORB::Subscription hil_imu_sub{ORB_ID(hil_sensor_imu)};
	uORB::Subscription hil_mag_baro_sub{ORB_ID(hil_sensor_mag_baro)};

	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	_is_running = true;

	// Track last published timestamps - use actual time only, never synthetic
	hrt_abstime last_imu_publish_ts = 0;
	hrt_abstime last_mag_publish_ts = 0;
	hrt_abstime last_baro_publish_ts = 0;

	// Temperature for IMU
	float imu_temperature = NAN;

	while (!_task_should_exit) {

		// ========== IMU ==========
		// Read latest HIL sensor data from apps proc (bridged via MUORB)
		hil_sensor_imu_s hil_imu_data;

		if (hil_imu_sub.update(&hil_imu_data)) {
			x_accel = hil_imu_data.accelerometer_m_s2[0];
			y_accel = hil_imu_data.accelerometer_m_s2[1];
			z_accel = hil_imu_data.accelerometer_m_s2[2];
			x_gyro = hil_imu_data.gyroscope_rad_s[0];
			y_gyro = hil_imu_data.gyroscope_rad_s[1];
			z_gyro = hil_imu_data.gyroscope_rad_s[2];

			if (PX4_ISFINITE(hil_imu_data.temperature)) {
				imu_temperature = hil_imu_data.temperature;
			}

			got_first_imu_msg = true;
			hil_imu_counter++;

			// Only publish when we have new data with finite values
			if (PX4_ISFINITE(x_accel) && PX4_ISFINITE(y_accel) && PX4_ISFINITE(z_accel) &&
			    PX4_ISFINITE(x_gyro) && PX4_ISFINITE(y_gyro) && PX4_ISFINITE(z_gyro)) {

				// Get current time - MUST be strictly greater than last
				hrt_abstime ts = hrt_absolute_time();

				// Only publish if time has actually advanced
				if (ts > last_imu_publish_ts) {
					// Publish gyro - set BOTH timestamps to the SAME value
					sensor_gyro_s gyro{};
					gyro.timestamp_sample = ts;
					gyro.device_id = 1310988; // DRV_IMU_DEVTYPE_SIM
					gyro.x = x_gyro;
					gyro.y = y_gyro;
					gyro.z = z_gyro;
					gyro.temperature = imu_temperature;
					gyro.error_count = 0;
					gyro.samples = 1;
					gyro.timestamp = ts;  // SAME as timestamp_sample
					_sensor_gyro_pub.publish(gyro);

					// Publish accel - set BOTH timestamps to the SAME value
					sensor_accel_s accel{};
					accel.timestamp_sample = ts;
					accel.device_id = 1310988; // DRV_IMU_DEVTYPE_SIM
					accel.x = x_accel;
					accel.y = y_accel;
					accel.z = z_accel;
					accel.temperature = imu_temperature;
					accel.error_count = 0;
					accel.samples = 1;
					accel.timestamp = ts;  // SAME as timestamp_sample
					_sensor_accel_pub.publish(accel);

					last_imu_publish_ts = ts;
					imu_counter++;
				}
			}
		}

		// ========== MAG/BARO ==========
		hil_sensor_mag_baro_s hil_mag_baro_data;

		if (hil_mag_baro_sub.update(&hil_mag_baro_data)) {
			got_first_mag_baro_msg = true;
			hil_mag_baro_counter++;

			hrt_abstime now = hrt_absolute_time();

			// Only publish magnetometer when mag field was actually updated
			if ((hil_mag_baro_data.fields_updated & hil_sensor_mag_baro_s::FIELD_MAG) &&
			    now > last_mag_publish_ts) {

				x_mag = hil_mag_baro_data.magnetometer_ga[0];
				y_mag = hil_mag_baro_data.magnetometer_ga[1];
				z_mag = hil_mag_baro_data.magnetometer_ga[2];
				temperature = hil_mag_baro_data.temperature;

				if (PX4_ISFINITE(x_mag) && PX4_ISFINITE(y_mag) && PX4_ISFINITE(z_mag)) {
					sensor_mag_s sensor_mag{};
					sensor_mag.timestamp_sample = now;
					// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
					sensor_mag.device_id = 197388;
					sensor_mag.x = x_mag;
					sensor_mag.y = y_mag;
					sensor_mag.z = z_mag;
					sensor_mag.temperature = PX4_ISFINITE(temperature) ? temperature : NAN;
					sensor_mag.error_count = 0;
					sensor_mag.timestamp = now;

					_sensor_mag_pub.publish(sensor_mag);
					last_mag_publish_ts = now;
					mag_counter++;
				}
			}

			// Only publish barometer when baro field was actually updated
			if ((hil_mag_baro_data.fields_updated & hil_sensor_mag_baro_s::FIELD_BARO) &&
			    now > last_baro_publish_ts) {

				pressure_pa = hil_mag_baro_data.pressure_pa;
				temperature = hil_mag_baro_data.temperature;

				if (PX4_ISFINITE(pressure_pa)) {
					sensor_baro_s sensor_baro{};
					sensor_baro.timestamp_sample = now;
					// 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
					sensor_baro.device_id = 6620172;
					sensor_baro.pressure = pressure_pa;
					sensor_baro.temperature = PX4_ISFINITE(temperature) ? temperature : 25.0f;
					sensor_baro.error_count = 0;
					sensor_baro.timestamp = now;

					_sensor_baro_pub.publish(sensor_baro);
					last_baro_publish_ts = now;
					baro_counter++;
				}
			}
		}

		// Check vehicle status
		bool vehicle_updated = false;
		(void) orb_check(_vehicle_status_sub, &vehicle_updated);

		if (vehicle_updated) {
			orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		}

		// Poll at ~250Hz to reduce MUORB load
		// HITL doesn't need ultra-low latency like real hardware
		usleep(4000);
	}

	orb_unsubscribe(_vehicle_status_sub);

	got_first_imu_msg = false;
	got_first_mag_baro_msg = false;
	_is_running = false;
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("dsp_hitl__main",
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

void usage()
{
	PX4_INFO("Usage: dsp_hitl {start|status|clear|stop}");
}

void clear_status_counters()
{
	hil_imu_counter = 0;
	hil_mag_baro_counter = 0;
	imu_counter = 0;
	mag_counter = 0;
	baro_counter = 0;
}

int get_status()
{
	PX4_INFO("Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("=== IMU ===");
	PX4_INFO("  HIL sensor_imu received: %u", hil_imu_counter);
	PX4_INFO("  IMU updates published: %u", imu_counter);
	PX4_INFO("  Current accel x, y, z: %f, %f, %f", double(x_accel), double(y_accel), double(z_accel));
	PX4_INFO("  Current gyro x, y, z: %f, %f, %f", double(x_gyro), double(y_gyro), double(z_gyro));
	PX4_INFO("=== MAG/BARO ===");
	PX4_INFO("  HIL sensor_mag_baro received: %u", hil_mag_baro_counter);
	PX4_INFO("  MAG updates published: %u", mag_counter);
	PX4_INFO("  BARO updates published: %u", baro_counter);
	PX4_INFO("  Current mag x, y, z: %f, %f, %f", double(x_mag), double(y_mag), double(z_mag));
	PX4_INFO("  Current pressure: %f Pa", double(pressure_pa));
	PX4_INFO("  Current temperature: %f C", double(temperature));

	return 0;
}

} // namespace dsp_hitl

int dsp_hitl_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		dsp_hitl::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return dsp_hitl::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return dsp_hitl::stop();
	}

	else if (!strcmp(verb, "status")) {
		return dsp_hitl::get_status();
	}

	else if (!strcmp(verb, "clear")) {
		dsp_hitl::clear_status_counters();
		return 0;
	}

	else {
		dsp_hitl::usage();
		return 1;
	}
}
