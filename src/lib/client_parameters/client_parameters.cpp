/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <string.h>
#include <stdbool.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/param_get_request.h>
#include <uORB/topics/param_get_retval.h>

#include "client_parameters.h"
#include "uORB/uORBManager.hpp"

static px4_task_t   _param_get_thread;
static px4_task_t   _param_retval_thread;

void _param_get_req(){
    int param_get_req_fd = orb_subscribe(ORB_ID(param_get_request));
    struct param_get_request_s  get_req;
    orb_advert_t param_get_h = nullptr;

    bool updated = false;

    while (true) {
        usleep(10000);
        (void) orb_check(param_get_req_fd, &updated);

        if (updated) {

            PX4_INFO("Got parameter get request");
            orb_copy(ORB_ID(param_get_request), param_get_req_fd, &get_req);
            if (param_get_h == nullptr) {
                param_get_h = orb_advertise(ORB_ID(param_get_request), &get_req);
            } else {
                orb_publish(ORB_ID(param_get_request), param_get_h, &get_req);
            }
        }
    }
}

void _param_get_retval(){
    int param_get_retval_fd = orb_subscribe(ORB_ID(param_get_retval));
    struct param_get_retval_s  get_retval;
    orb_advert_t param_retval_h = nullptr;

    bool updated = false;
    while (true) {
        usleep(10000);
        (void) orb_check(param_get_retval_fd, &updated);

        if (updated) {

            PX4_INFO("Got parameter get retval");
            orb_copy(ORB_ID(param_get_retval), param_get_retval_fd, &get_retval);
            if (param_retval_h == nullptr) {
                param_retval_h = orb_advertise(ORB_ID(param_get_retval), &get_retval);
            }
        }
    }
}


void
_initialize_param_client()
{
        _param_get_thread = px4_task_spawn_cmd("param_client_get_request",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_PARAMS,
                                             (1024 * 4),
                                             (px4_main_t) _param_get_req,
                                             NULL);

        _param_retval_thread = px4_task_spawn_cmd("param_client_get_retval",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_PARAMS,
                                             (1024 * 4),
                                             (px4_main_t) _param_get_retval,
                                             NULL);
}

