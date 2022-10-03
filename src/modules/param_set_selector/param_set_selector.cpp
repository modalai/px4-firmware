/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Daniel M. Sahu <>
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class ParamSetSelector : public ModuleBase<ParamSetSelector>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ParamSetSelector();
	~ParamSetSelector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** perform all initialization processes */
	bool init();

private:
	/**< hardcoded parameter sets */
	enum class ParameterSet {
		DISABLED = 0,
                ACRO_FAST = 1,
                ALT_FAST = 2,
                ALT_SLOW = 3,
                RESERVED4 = 4,
                RESERVED5 = 5
	};

	/** Core loop method. */
	void Run() override;

	/** Check for changes in parameters. */
	void parameter_update_poll();

	/** switch to the given set. */
	void switchSet(const ParameterSet& set);

	/**< notification of parameter updates */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	/**< loop performance counter */
	perf_counter_t	_loop_perf;

	/**< define *our* parameters */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PARAM_SET>) _param_param_set
	)
};

ParamSetSelector::ParamSetSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	parameter_update_poll();
}

ParamSetSelector::~ParamSetSelector()
{
	ScheduleClear();
}

void ParamSetSelector::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// update parameter set
		switchSet(static_cast<ParameterSet>(_param_param_set.get()));
	}
}

void ParamSetSelector::switchSet(const ParameterSet& set)
{
	// switch to the new parameter set
	switch (set)
	{
		case ParameterSet::DISABLED:
			break;
		case ParameterSet::ACRO_FAST:
		{
			const uint tilt = 60;
			param_set(param_find("MPC_MAN_TILT_MAX"), &tilt);
			break;
		}
		case ParameterSet::ALT_FAST:
		{
			const uint tilt = 40;
			param_set(param_find("MPC_MAN_TILT_MAX"), &tilt);
			break;
		}
		case ParameterSet::ALT_SLOW:
		{
			const uint tilt = 20;
			param_set(param_find("MPC_MAN_TILT_MAX"), &tilt);
			break;
		}
		case ParameterSet::RESERVED4:
			break;
		case ParameterSet::RESERVED5:
			break;
		default:
			// should be unreachable...
			break;
	}
}

void ParamSetSelector::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// begin perf counter
	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	// schedule next iteration
	ScheduleDelayed(100_ms);

	// end perf counter
	perf_end(_loop_perf);
}

int ParamSetSelector::task_spawn(int argc, char *argv[])
{
	ParamSetSelector *instance = new ParamSetSelector();

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

bool ParamSetSelector::init()
{
	ScheduleNow();
	return true;
}

int ParamSetSelector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ParamSetSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

This module toggles between disjoint collections (sets) of parameters.

### Implementation

Monitors the 'PARAM_SET' parameter. If the parameter is updated, switch
to the set defined by that parameter and update all corresponding parameters.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("param_set_selector", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int param_set_selector_main(int argc, char *argv[])
{
	return ParamSetSelector::main(argc, argv);
}
