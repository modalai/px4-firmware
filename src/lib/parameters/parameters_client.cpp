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

#include "param.h"
#include <pthread.h>
#include <vector>

#include <px4_platform_common/log.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_request.h>
#include <uORB/topics/parameter_value.h>

std::vector<const char*> param_vector;
uORB::Publication<parameter_request_s> _param_response_pub{ORB_ID(parameter_request)};
uORB::Subscription _param_value_sub{ORB_ID(parameter_value)};
pthread_mutex_t lock;

param_t param_find(const char *name)
{
	pthread_mutex_lock(&lock);

	auto it = find(param_vector.begin(), param_vector.end(), name);

	if(it != param_vector.end()){
		param_t index = it - param_vector.begin();
		return index;
	} else {
		param_vector.push_back(name);
		param_t index = param_vector.size() - 1;
		return index;
	}

	pthread_mutex_unlock(&lock);

	return EINVAL;
}

int param_get(param_t param, void *val)
{
	pthread_mutex_lock(&lock);
	const char* param_name = param_vector[param];

	parameter_request_s request{};

	strcpy(request.name, param_name);
	request.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ;
	_param_response_pub.publish(request);

	while(true){
                usleep(10000);
		if (_param_value_sub.updated()) {

			parameter_value_s value;
			if (_param_value_sub.copy(&value)) {

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
	}
	pthread_mutex_unlock(&lock);

	return PX4_ERROR;

}

int param_set_no_notification(param_t param, const void *val)
{
	PX4_INFO("INSIDE param_set_no_notification");
	return -1;
}

void param_notify_changes()
{
	PX4_INFO("INSIDE param_notify_changes");
	return;
}