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

size_t ParameterClient::getParameterSize(param_t param)
{
	if (handle_in_range(param)) {
		switch (getParameterType(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		default:
			return 0;
		}
	}

	return 0;
}

param_t ParameterClient::findParameter(const char *name, bool notification)
{
	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	PX4_INFO("Looking for parameter %s", name);

	//perform a binary search of the known parameters
	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, getParameterName(middle));
		if (ret == 0) {
			PX4_INFO("Found parameter %s", name);
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
	PX4_ERR("Couldn't find parameter %s", name);
	return PARAM_INVALID;
}

int ParameterClient::getParameterValue(param_t param, void *val)
{
	if (_param_value_sub == PX4_ERROR) {
		_param_value_sub = orb_subscribe(ORB_ID(parameter_value));
		if (_param_value_sub == PX4_ERROR) {
			PX4_ERR("Couldn't subscribe to parameter_value");
		} else {
			PX4_INFO("Successfully subscribed to parameter_value");
		}
		return PX4_ERROR;
	}

	LockGuard lg{lock};

	const char* param_name = getParameterName(param);
	if (param_name == nullptr){
		PX4_ERR("Couldn't get parameter name");
		return PX4_ERROR;
	}

	//PX4_INFO("^^^ Getting parameter %s ^^^", param_name);

	parameter_request_s request{};
	strncpy(request.name, param_name, sizeof(request.name));
	request.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ;

	_param_request_pub.publish(request);

	parameter_value_s value;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _param_value_sub;
	fds[0].events = POLLIN;

	int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 5000);

	if (pret > 0 && fds[0].revents & POLLIN) {
		orb_copy(ORB_ID(parameter_value), _param_value_sub, &value);
		switch (value.type) {
			case parameter_request_s::TYPE_UINT8:
			case parameter_request_s::TYPE_INT32:
			case parameter_request_s::TYPE_INT64:
			{
				memcpy(val, &value.int64_value, getParameterSize(param));
				//PX4_INFO("^^^ Got int64 value %d ^^^", value.int64_value);
				return PX4_OK;

			}

			case parameter_request_s::TYPE_FLOAT32:
			case parameter_request_s::TYPE_FLOAT64: {
				memcpy(val, &value.float64_value, getParameterSize(param));
				//PX4_INFO("^^^ Got float64 value %d ^^^", value.float64_value);
				return PX4_OK;

			}
			default:
				PX4_ERR("Got unknown data type %d", value.type);
				break;
		}
	} else {
		PX4_ERR("Got timeout on px4_poll. Return value: %d", pret);
	}

	return PX4_ERROR;
}

int ParameterClient::setParameter(param_t param, const void *val, bool notify_changes)
{
	const char* param_name = getParameterName(param);
	if(param_name == nullptr){
		return PX4_ERROR;
	}

	parameter_request_s request_change{};

	strncpy(request_change.name, param_name, sizeof(request_change.name));
	request_change.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_SET;
	request_change.type = getParameterType(param);
	request_change.notify_changes = notify_changes;

	if(request_change.type == PARAM_TYPE_UNKNOWN){
		return PX4_ERROR;
	}

	switch (request_change.type) {
		case PARAM_TYPE_INT32:
		{
			request_change.type = parameter_request_s::TYPE_INT64;
			memcpy(&request_change.int64_value, val, getParameterSize(param));
			int retval = _param_request_pub.publish(request_change);
			if(retval){
				return PX4_OK;
			}
		}
		case PARAM_TYPE_FLOAT:
		{
			request_change.type = parameter_request_s::TYPE_FLOAT64;
			memcpy(&request_change.float64_value, val, getParameterSize(param));
			int retval = _param_request_pub.publish(request_change);
			if(retval){
				return PX4_OK;
			}		}
		default:
			break;

	}

	return PX4_ERROR;
}

void ParameterClient::notifyChanges()
{
	return;
}
