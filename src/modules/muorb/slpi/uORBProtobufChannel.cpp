/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
#include "uORBProtobufChannel.hpp"
#include <string.h>
#include <stdio.h>
#include <qurt.h>
#include <qurt_thread.h>

#define PX4_TASK_STACK_SIZE 8192
#define PX4_TASK_MAX_NAME_LENGTH 32
#define PX4_TASK_MAX_ARGC 32
#define PX4_TASK_MAX_ARGV_LENGTH 32
#define PX4_MAX_TASKS 24

extern "C" void HAP_debug(const char *msg, int level, const char *filename, int line);

fc_func_ptrs muorb_func_ptrs;
char name[PX4_TASK_MAX_NAME_LENGTH + 4] = "send_back_thread";
char stack[PX4_TASK_STACK_SIZE];
int priority = 0;
bool assigned_bool = false;

qurt_thread_t tid;
qurt_thread_attr_t attr;

int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs, int32_t clock_offset_us)
{
	if (func_ptrs == NULL) {
		return -1;

	} else {
		if (!assigned_bool) {
			HAP_debug("func_ptrs assigned!", 1, "test", 0);
			muorb_func_ptrs = *func_ptrs;
			assigned_bool = true;
		}

		if ((muorb_func_ptrs.advertise_func_ptr == NULL) ||
		    (muorb_func_ptrs.subscribe_func_ptr == NULL) ||
		    (muorb_func_ptrs.unsubscribe_func_ptr == NULL) ||
		    (muorb_func_ptrs.topic_data_func_ptr == NULL) ||
		    (muorb_func_ptrs.register_interrupt_callback == NULL)) {
			HAP_debug("muorb equals null pointers", 1, "test", 0);
			return -1;
		}
	}

	return 0;
}

int px4muorb_topic_advertised(const char *topic_name)
{
	HAP_debug(topic_name, 1, "px4muorb_topic_advertised", 0);
	qurt_thread_attr_init(&attr);
	qurt_thread_attr_set_name(&attr, name);
	qurt_thread_attr_set_stack_addr(&attr, stack);
	qurt_thread_attr_set_stack_size(&attr, PX4_TASK_STACK_SIZE);
	qurt_thread_attr_set_priority(&attr, 0);
	(void) qurt_thread_create(&tid, &attr, &send_helper, 0);
	HAP_debug("Thread being created", 1, "qurt_thread_info", 0);
	return 0;
}

static void send_helper(void *)
{
	char hello_world_message[] = "Hello, World!";
	HAP_debug("Send Helper called", 1, "qurt_thread_info", 0);
	send_message("slpi_debug", strlen(hello_world_message) + 1, (uint8_t *) hello_world_message);
	qurt_thread_exit(0);
}

int16_t send_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = muorb_func_ptrs.topic_data_func_ptr(messageName, data, length);
	return rc;
}

int px4muorb_add_subscriber(const char *topic_name)
{
	HAP_debug(topic_name, 1, "px4muorb_add_subscriber", 0);
	return 0;
}

int px4muorb_remove_subscriber(const char *topic_name)
{
	HAP_debug(topic_name, 1, "px4muorb_remove_subscriber", 0);
	return 0;
}

int px4muorb_send_topic_data(const char *topic_name, const uint8_t *data,
			     int data_len_in_bytes)
{
	HAP_debug(topic_name, 1, "px4muorb_send_topic_data", 0);
	return 0;
}
