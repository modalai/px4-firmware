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

#define PARAM_CLIENT_CALLBACK_IMPLEMENTATION

#include "ParameterClient.hpp"
static ParameterClient *parameter_client {nullptr};

#include "param.h"

void param_init()
{
	if (parameter_client == nullptr) {
		parameter_client = new ParameterClient();
	}
}

void param_notify_changes()
{
	if (parameter_client) {
		parameter_client->notifyChanges();
	}
}

param_t param_find(const char *name)
{
	if (parameter_client) {
		return parameter_client->findParameter(name, true);
	}

	return PARAM_INVALID;
}

param_t param_find_no_notification(const char *name)
{
	PX4_ERR("param_find_no_notification called for parameter %s", name);

	return param_find(name);
}


int param_get(param_t param, void *val)
{
	if (parameter_client) {
		return parameter_client->getParameterValue(param, val);
	}

	return -1;
}

void param_set_used(param_t param)
{
	if (parameter_client) {
		const char* param_name = parameter_client->getParameterName(param);
		if (param_name != nullptr) {
			PX4_ERR("param_set_used called for parameter %s", param_name);
		} else {

			PX4_ERR("param_set_used called for unknown parameter");
		}
	} else {
		PX4_ERR("param_set_used called before initialization");
	}
}


int param_set(param_t param, const void *val)
{
	if (parameter_client) {
		return parameter_client->setParameter(param, val, true);
	}

	return -1;
}

int param_set_no_notification(param_t param, const void *val)
{
	if (parameter_client) {
		return parameter_client->setParameter(param, val, false);
	}

	return -1;
}

int param_reset_no_notification(param_t param)
{
	if (parameter_client) {
		const char* param_name = parameter_client->getParameterName(param);
		if (param_name != nullptr) {
			PX4_ERR("param_reset_no_notification called for parameter %s", param_name);
		} else {
			PX4_ERR("param_reset_no_notification called for unknown parameter");
		}
	} else {
		PX4_ERR("param_reset_no_notification called before initialization");
	}
	return -1;
}
