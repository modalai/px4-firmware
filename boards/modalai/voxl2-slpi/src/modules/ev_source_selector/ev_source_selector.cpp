/****************************************************************************
 *
 *   Copyright (c) 2024 ModalAI, inc. All rights reserved.
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

#include <px4_log.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_status.h>

class EV_SourceSelector : public ModuleBase<EV_SourceSelector>, public px4::WorkItem
{
public:

	// EV_SourceSelector(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
	// 	SubscriptionCallback(meta, interval_us, instance)
	// {
	// }
	// 
	// virtual ~EV_SourceSelector()
	// {
	// 	unregisterCallback();
	// };

	EV_SourceSelector();
	~EV_SourceSelector() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	// void call() override
	// {
	// 	PX4_INFO("Got vehicle status update callback");
	// 	if (updated()) {
	// 		uint8_t _previous_nav_state_user_intention = _vehicle_status.nav_state_user_intention;
	// 		if (update(&_vehicle_status)) {
	// 			if (_vehicle_status.nav_state_user_intention != _previous_nav_state_user_intention) {
	// 				PX4_INFO("Intended flight mode changed!!!");
	// 			}
	// 		} else {
	// 			PX4_ERR("vehicle status update failed");
	// 		}
	// 	} else {
	// 		PX4_INFO("Got callback but there was no update!!!");
	// 	}
	// }

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _status_sub{this, ORB_ID(vehicle_status)};

	vehicle_status_s _vehicle_status{};
	int32_t _ev_ctrl;

};

EV_SourceSelector::EV_SourceSelector() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool EV_SourceSelector::init()
{
	if (!_status_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void EV_SourceSelector::Run()
{
	if (should_exit()) {
		_status_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_status_sub.updated()) {
		uint8_t _previous_nav_state_user_intention = _vehicle_status.nav_state_user_intention;
		if (_status_sub.update(&_vehicle_status)) {
			uint8_t new_nav_state_user_intention = _vehicle_status.nav_state_user_intention;
			if (new_nav_state_user_intention != _previous_nav_state_user_intention) {
				param_get(param_find("EKF2_EV_CTRL"), &_ev_ctrl);
				PX4_INFO("Intended flight mode changed");
				if ((new_nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_POSCTL) ||
					(new_nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
					if (_ev_ctrl != 15) {
						int new_ctrl_val = 15;
						PX4_INFO("Changing EKF2_EV_CTRL to 15");
						param_set(param_find("EKF2_EV_CTRL"), (const void*) &new_ctrl_val);
					}
				} else {
					if (_ev_ctrl != 0) {
						int new_ctrl_val = 0;
						PX4_INFO("Changing EKF2_EV_CTRL to 0");
						param_set(param_find("EKF2_EV_CTRL"), (const void*) &new_ctrl_val);
					}
				}
			}
		}
	}
}

int EV_SourceSelector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EV_SourceSelector::task_spawn(int argc, char *argv[])
{
	EV_SourceSelector *instance = new EV_SourceSelector();

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

int EV_SourceSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude estimator q.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("EV_SourceSelector", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ev_source_selector_main(int argc, char *argv[])
{
	return EV_SourceSelector::main(argc, argv);
}
