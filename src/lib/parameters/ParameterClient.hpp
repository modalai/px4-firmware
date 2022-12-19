/****************************************************************************
 *
 *   Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/parameter_request.h>
#include <uORB/topics/parameter_value.h>

#include "param.h"
#include <parameters/px4_parameters.hpp>
#include <containers/LockGuard.hpp>

using namespace time_literals;

class ParameterClient
{
public:
	ParameterClient();
	~ParameterClient();

	/**
	 * Look up a parameter by name.
	 *
	 * @param name		The canonical name of the parameter being looked up.
	 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
	 */
	param_t findParameter(const char *name, bool notification = true);

	/**
	 * Obtain the name of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
	 */
	const char *getParameterName(param_t param);
	/**
	 * Obtain the type of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The type assigned to the parameter.
	 */
	param_type_t getParameterType(param_t param);

	/**
	 * Notify the system about parameter changes. Can be used for example after several calls to
	 * param_set_no_notification() to avoid unnecessary system notifications.
	 */
	void notifyChanges();

	/**
	 * Copy the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
	 */
	int getParameterValue(param_t param, void *val);

	/**
	 * Set the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		The value to set; assumed to point to a variable of the parameter type.
	 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
	 */
	int setParameter(param_t param, const void *val, bool notify_changes);

private:
	static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

	/**
	 * Test whether a param_t is value.
	 *
	 * @param param			The parameter handle to test.
	 * @return			True if the handle is valid.
	 */
	static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

	uORB::Publication<parameter_request_s> _param_request_pub{ORB_ID(parameter_request)};
	int _param_value_sub = orb_subscribe(ORB_ID(parameter_value));

	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
};
