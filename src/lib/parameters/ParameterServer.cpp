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

#define PARAM_IMPLEMENTATION

#include <parameters/px4_parameters.hpp>
#include "ParameterServer.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

ParameterServer::ParameterServer() :
	ScheduledWorkItem("parameter_server", px4::wq_configurations::parameter_server)
{
	_srv_parameter_get_request_sub.registerCallback();
	_srv_parameter_set_request_sub.registerCallback();

	ScheduleOnInterval(100_ms);
}

ParameterServer::~ParameterServer()
{
}

void ParameterServer::Run()
{
	// PX4_INFO("Running ParameterServer thread main loop");

	// srv: parameter_set
	if (_srv_parameter_set_request_sub.updated()) {
		const unsigned last_generation = _srv_parameter_set_request_sub.get_last_generation();

		srv_parameter_set_request_s request;

		if (_srv_parameter_set_request_sub.copy(&request)) {

			if (_srv_parameter_set_request_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("missed srv_parameter_set_request, generation %d -> %d", last_generation,
					_srv_parameter_set_request_sub.get_last_generation());
			}

			srv_parameter_set_response_s response{};
			response.timestamp_requested = request.timestamp;

			// defaults
			response.result = srv_parameter_set_response_s::RESULT_SET_FAILED;
			response.parameter.index = PARAM_INVALID;
			response.parameter.type = parameter_s::TYPE_INVALID;

			param_t param = PARAM_INVALID;

			if (handle_in_range(request.parameter.index) && (strlen(request.parameter.name) == 0)) {
				param = request.parameter.index;
				// PX4_INFO("ParameterServer got parameter_set_request for index %u (%s)", param, param_name(param));

			} else {
				param = param_find(request.parameter.name);
				// PX4_INFO("ParameterServer got parameter_set_request for %s", request.parameter.name);
			}

			response.parameter.index = param;

			if (param != PARAM_INVALID) {
				memcpy(response.parameter.name, param_name(param), 16);
				response.parameter.index_used = param_for_used_index(param);

				switch (param_type(param)) {
				case PARAM_TYPE_INT32:
					response.parameter.type = parameter_s::TYPE_INT32;

					if (param_set(param, &request.parameter.int32_value) == 0) {
						response.parameter.int32_value = request.parameter.int32_value;
						response.result = srv_parameter_set_response_s::RESULT_SET_SUCCESS;
					}

					break;

				case PARAM_TYPE_FLOAT:
					response.parameter.type = parameter_s::TYPE_FLOAT32;

					if (param_set(param, &request.parameter.float32_value) == 0) {
						response.parameter.float32_value = request.parameter.float32_value;
						response.result = srv_parameter_set_response_s::RESULT_SET_SUCCESS;
					}

					break;
				}

			} else {
				response.result = srv_parameter_set_response_s::RESULT_ERROR_INVALID_PARAMETER;
			}

			response.timestamp = hrt_absolute_time();
			_srv_parameter_set_response_pub.publish(response);
		}
	}

	// srv: parameter_get
	if (_srv_parameter_get_request_sub.updated()) {
		const unsigned last_generation = _srv_parameter_get_request_sub.get_last_generation();

		srv_parameter_get_request_s request;

		if (_srv_parameter_get_request_sub.copy(&request)) {

			if (_srv_parameter_get_request_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("missed srv_parameter_get_request, generation %d -> %d", last_generation,
					_srv_parameter_get_request_sub.get_last_generation());
			}

			srv_parameter_get_response_s response{};
			response.timestamp_requested = request.timestamp;

			// defaults
			response.result = srv_parameter_get_response_s::RESULT_GET_FAILED;
			response.parameter.index = PARAM_INVALID;
			response.parameter.index_used = -1;
			response.parameter.type = parameter_s::TYPE_INVALID;

			param_t param = PARAM_INVALID;

			if (handle_in_range(request.index) && (strlen(request.name) == 0)) {
				param = request.index;
				// PX4_INFO("ParameterServer got parameter_get_request for index %u (%s), generation %u",
				// 		 param, param_name(param), _srv_parameter_get_request_sub.get_last_generation());

			} else {
				param = param_find(request.name);
				// PX4_INFO("ParameterServer got parameter_get_request for %s, generation %u",
				// 		 request.name, _srv_parameter_get_request_sub.get_last_generation());
			}

			response.parameter.index = param;

			if (param != PARAM_INVALID) {
				memcpy(response.parameter.name, param_name(param), 16);
				response.parameter.index_used = param_for_used_index(param);

				switch (param_type(param)) {
				case PARAM_TYPE_INT32:
					response.parameter.type = parameter_s::TYPE_INT32;

					if (param_get(param, &response.parameter.int32_value) == 0) {
						response.result = srv_parameter_get_response_s::RESULT_GET_SUCCESS;
					}

					break;

				case PARAM_TYPE_FLOAT:
					response.parameter.type = parameter_s::TYPE_FLOAT32;

					if (param_get(param, &response.parameter.float32_value) == 0) {
						response.result = srv_parameter_get_response_s::RESULT_GET_SUCCESS;
					}

					break;
				}

			} else {
				response.result = srv_parameter_get_response_s::RESULT_ERROR_INVALID_PARAMETER;
			}

			response.timestamp = hrt_absolute_time();
			_srv_parameter_get_response_pub.publish(response);
		}
	}

	// schedule to run again immediately if there are pending updates
	if (_srv_parameter_set_request_sub.updated() || _srv_parameter_get_request_sub.updated()) {
		ScheduleNow();
		return;
	}

}
