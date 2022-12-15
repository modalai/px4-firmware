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

#define PARAM_CLIENT_IMPLEMENTATION

#include "ParameterClient.hpp"

ParameterClient::ParameterClient() {}

ParameterClient::~ParameterClient() {}

const char *ParameterClient::getParameterName(param_t param)
{
	if (handle_in_range(param)) {
		return px4::parameters[param].name;
	}
	return nullptr;
}

param_type_t ParameterClient::getParameterType(param_t param)
{
	return handle_in_range(param) ? px4::parameters_type[param] : PARAM_TYPE_UNKNOWN;
}

param_t ParameterClient::findParameter(const char *name, bool notification)
{
	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	//perform a binary search of the known parameters
	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, getParameterName(middle));
		if (ret == 0) {
			return middle;

		} else if (middle == front) {
			// An end point has been hit, but there has been no match
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	// not found
	return PARAM_INVALID;
}

int ParameterClient::getParameterValue(param_t param, void *val)
{
	const char* param_name = getParameterName(param);

	parameter_request_s request{};
	strcpy(request.name, param_name);
	request.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ;
	_param_response_pub.publish(request);
	while(true){
		parameter_value_s value;
		if (_param_value_sub.updateBlocking(value, 100000)) {

			switch (value.type) {
				case parameter_request_s::TYPE_UINT8:
				case parameter_request_s::TYPE_INT32:
				case parameter_request_s::TYPE_INT64:
				{
					memcpy(val, &value.int64_value, sizeof(value.int64_value));
					return PX4_OK;
				}

				case parameter_request_s::TYPE_FLOAT32:
				case parameter_request_s::TYPE_FLOAT64: {
					memcpy(val, &value.float64_value, sizeof(value.float64_value));
					return PX4_OK;

				}
			}
		}
	}
	return PX4_ERROR;
}

int ParameterClient::setParameter(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	const char* param_name = getParameterName(param);
	parameter_request_s request_change{};

	strcpy(request_change.name, param_name);
	request_change.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_SET;
	request_change.type = getParameterType(param);

	switch (request_change.type) {
		case PARAM_TYPE_INT32:
		{
			request_change.type = parameter_request_s::TYPE_INT64;
			memcpy(&request_change.int64_value, val, sizeof(request_change.int64_value));
			_param_response_pub.publish(request_change);
			return PX4_OK;
		}
		case PARAM_TYPE_FLOAT:
		{
			request_change.type = parameter_request_s::TYPE_FLOAT64;
			memcpy(&request_change.float64_value, val, sizeof(request_change.float64_value));
			_param_response_pub.publish(request_change);
			return PX4_OK;
		}
	}
	return PX4_ERROR;
}

void ParameterClient::notifyChanges()
{
	return;
}
