/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

// #include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
// #include <px4_platform_common/atomic_bitset.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/posix.h>
// #include <px4_platform_common/sem.h>
// #include <px4_platform_common/shutdown.h>
// #include <containers/Bitset.hpp>
// #include <drivers/drv_hrt.h>
// #include <lib/perf/perf_counter.h>

// #include "tinybson/tinybson.h"
// #include "uthash/utarray.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

// #include <uORB/topics/actuator_armed.h>
// #include <uORB/topics/parameter_request.h>
// #include <uORB/topics/parameter_value.h>
// #include <uORB/topics/parameter_update.h>

#include <uORB/topics/srv_parameter_get_request.h>
#include <uORB/topics/srv_parameter_get_response.h>
#include <uORB/topics/srv_parameter_set_request.h>
#include <uORB/topics/srv_parameter_set_response.h>

// #include <px4_platform_common/param.h>

class ParameterServer : public px4::ScheduledWorkItem
{
public:
	ParameterServer();
	~ParameterServer() override;

private:

	static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

	/**
	 * Test whether a param_t is valid.
	 *
	 * @param param			The parameter handle to test.
	 * @return			True if the handle is valid.
	 */
	static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

	void Run() override;

	// srv: parameter_get
	uORB::SubscriptionCallbackWorkItem _srv_parameter_get_request_sub{this, ORB_ID(srv_parameter_get_request)};
	uORB::Publication<srv_parameter_get_response_s> _srv_parameter_get_response_pub{ORB_ID(srv_parameter_get_response)};

	// srv: parameter_set
	uORB::SubscriptionCallbackWorkItem _srv_parameter_set_request_sub{this, ORB_ID(srv_parameter_set_request)};
	uORB::Publication<srv_parameter_set_response_s> _srv_parameter_set_response_pub{ORB_ID(srv_parameter_set_response)};
};
