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

#include "worker_thread.hpp"
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "calibration_routines.h"
#include "baro_calibration.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "level_calibration.h"
#include "mag_calibration.h"
#include "rc_calibration.h"

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>
#include <parameters/param.h>


using namespace time_literals;

WorkerThread::WorkerThread()
{
	px4_sem_init(&_work_sem, 0, 0);
	px4_sem_init(&_exit_sem, 0, 0);
}

WorkerThread::~WorkerThread()
{
	if (_task_started) {
		// Signal shutdown and wake the thread
		_shutdown.store(true);
		px4_sem_post(&_work_sem);

		// Wait for thread to signal it has exited
		px4_sem_wait(&_exit_sem);
	}

	px4_sem_destroy(&_work_sem);
	px4_sem_destroy(&_exit_sem);
}

void WorkerThread::startTask(Request request)
{
	if (isBusy()) {
		return;
	}

	_request = request;
	_state.store((int)State::Running);

	if (!_task_started) {
		// First time: create the persistent task
		_task_handle = px4_task_spawn_cmd(
					"commander_worker",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT - 50,
					PX4_STACK_ADJUSTED(4804),
					&WorkerThread::taskTrampoline,
					(char *const *)this);

		if (_task_handle < 0) {
			PX4_ERR("px4_task_spawn_cmd failed: %d", _task_handle);
			_state.store((int)State::Finished);
			_ret_value = _task_handle;
			return;
		}

		_task_started = true;
	}

	// Wake the worker thread
	px4_sem_post(&_work_sem);
}

int WorkerThread::taskTrampoline(int argc, char *argv[])
{
	// Recover 'this' pointer passed via argv
	WorkerThread *worker = reinterpret_cast<WorkerThread *>(argv);
	worker->workerLoop();
	return 0;
}

void WorkerThread::workerLoop()
{
	while (!_shutdown.load()) {
		// Wait for work
		px4_sem_wait(&_work_sem);

		if (_shutdown.load()) {
			break;
		}

		// Execute the requested task
		switch (_request) {
		case Request::GyroCalibration:
			_ret_value = do_gyro_calibration(&_mavlink_log_pub);
			break;

		case Request::MagCalibration:
			_ret_value = do_mag_calibration(&_mavlink_log_pub);
			break;

		case Request::RCTrimCalibration:
			_ret_value = do_trim_calibration(&_mavlink_log_pub);
			break;

		case Request::AccelCalibration:
			_ret_value = do_accel_calibration(&_mavlink_log_pub);
			break;

		case Request::LevelCalibration:
			_ret_value = do_level_calibration(&_mavlink_log_pub);
			break;

		case Request::AccelCalibrationQuick:
			_ret_value = do_accel_calibration_quick(&_mavlink_log_pub);
			break;

		case Request::AirspeedCalibration:
			_ret_value = do_airspeed_calibration(&_mavlink_log_pub);
			break;

		case Request::ESCCalibration:
			_ret_value = do_esc_calibration(&_mavlink_log_pub);
			break;

		case Request::MagCalibrationQuick:
			_ret_value = do_mag_calibration_quick(&_mavlink_log_pub, _heading_radians, _latitude, _longitude);
			break;

		case Request::BaroCalibration:
			_ret_value = do_baro_calibration(&_mavlink_log_pub);
			break;

		case Request::ParamLoadDefault:
			_ret_value = param_load_default();

			if (_ret_value != 0) {
				mavlink_log_critical(&_mavlink_log_pub, "Error loading settings\t");
				events::send(events::ID("commander_load_param_failed"), events::Log::Critical, "Error loading settings");
			}

			break;

		case Request::ParamSaveDefault:
			_ret_value = param_save_default();

			if (_ret_value != 0) {
				mavlink_log_critical(&_mavlink_log_pub, "Error saving settings\t");
				events::send(events::ID("commander_save_param_failed"), events::Log::Critical, "Error saving settings");
			}

			break;

		case Request::ParamResetAll:
			param_reset_all();
			_ret_value = 0;
			break;

		case Request::ParamResetSensorFactory: {
				const char *reset_cal[] = { "CAL_ACC*", "CAL_GYRO*", "CAL_MAG*" };
				param_reset_specific(reset_cal, sizeof(reset_cal) / sizeof(reset_cal[0]));
				_ret_value = param_save_default();
#if defined(CONFIG_BOARDCTL_RESET)
				px4_reboot_request(false, 400_ms);
#endif // CONFIG_BOARDCTL_RESET
				break;
			}

		case Request::ParamResetAllConfig: {
				const char *exclude_list[] = {
					"LND_FLIGHT_T_HI",
					"LND_FLIGHT_T_LO",
					"COM_FLIGHT_UUID"
				};
				param_reset_excludes(exclude_list, sizeof(exclude_list) / sizeof(exclude_list[0]));
				_ret_value = 0;
				break;
			}
		}

		_state.store((int)State::Finished);
	}

	// Signal destructor that we've exited
	px4_sem_post(&_exit_sem);
}

void WorkerThread::setMagQuickData(float heading_rad, float lat, float lon)
{
	_heading_radians = heading_rad;
	_latitude = lat;
	_longitude = lon;
}
