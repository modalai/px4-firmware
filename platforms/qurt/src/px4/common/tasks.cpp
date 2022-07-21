/****************************************************************************
 *
 *   Copyright (C) 2015-2016 Mark Charlebois. All rights reserved.
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
 * @file px4_qurt_tasks.c
 * Implementation of existing task API for QURT.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/time.h>
#include <hrt_work.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

#if !defined(__PX4_QURT)
#include <signal.h>
#endif

#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>

#include <qurt.h>

#define PX4_TASK_STACK_SIZE 8192
#define PX4_TASK_MAX_NAME_LENGTH 32
#define PX4_TASK_MAX_ARGC 32
#define PX4_TASK_MAX_ARGV_LENGTH 32
#define PX4_MAX_TASKS 24

#ifdef __PX4_QURT
typedef void* (*px4_qurt_task_func_t)(void *arg);
#endif

typedef struct task_entry {
	qurt_thread_t tid;
	char name[PX4_TASK_MAX_NAME_LENGTH + 4];
    char stack[PX4_TASK_STACK_SIZE];
    qurt_thread_attr_t attr;
	px4_main_t main_entry;
	px4_qurt_task_func_t arg_entry;
    void *arg;
	int argc;
	char argv_storage[PX4_TASK_MAX_ARGC][PX4_TASK_MAX_ARGV_LENGTH];
	char *argv[PX4_TASK_MAX_ARGC];
	bool isused;

	task_entry() : isused(false) {}
} task_entry_t;

static task_entry_t taskmap[PX4_MAX_TASKS];

static bool task_mutex_initialized = false;
static qurt_mutex_t task_mutex;

static void entry_adapter(void *ptr)
{
	task_entry_t *data;
	data = (task_entry_t*) ptr;

	if (data->main_entry) data->main_entry(data->argc, data->argv);
	else if (data->arg_entry) data->arg_entry(data->arg);
    else PX4_ERR("No valid task entry points");

    qurt_thread_exit(QURT_EOK);
}

static px4_task_t px4_task_spawn_internal(const char *name, int priority,
                                          px4_main_t main_entry,
			                              char *const argv[],
                                          px4_qurt_task_func_t arg_entry,
                   				          void *arg) {
	int retcode = 0;
	int i = 0;
	int task_index = 0;
	char *p = (char *)argv;

	PX4_INFO("Creating qurt thread %s\n", name);

    if ((main_entry != nullptr) && (arg_entry != nullptr)) {
        PX4_ERR("Can only have one type of entry function");
        return -1;
    }

    // This part is not thread safe but it shouldn't
    // matter because the dspal task starts before everything
    // else and will successfully initialize the mutex for later.
    if (task_mutex_initialized == false) {
        task_mutex_initialized = true;
        qurt_mutex_init(&task_mutex);
    }

    qurt_mutex_lock(&task_mutex);

    // Get a free task structure
    for (task_index = 0; task_index < PX4_MAX_TASKS; task_index++) {
        if (taskmap[task_index].isused == false) break;
    }
    if (task_index == PX4_MAX_TASKS) {
        qurt_mutex_unlock(&task_mutex);
        PX4_ERR("Hit maximum number of threads");
        return -1;
    }

	// Calculate argc
	taskmap[task_index].argc = 0;
	while (p) {
		taskmap[task_index].argc++;
		p = argv[taskmap[task_index].argc];
	}
    if (taskmap[task_index].argc >= PX4_TASK_MAX_ARGC) {
        qurt_mutex_unlock(&task_mutex);
        PX4_ERR("Too many arguments for thread %d", taskmap[task_index].argc);
        return -1;
    }

    // Copy arguments into our static storage and setup argv array
    for (i = 0; i < PX4_TASK_MAX_ARGC; i++) {
        if (i < taskmap[task_index].argc) {
            int argument_length = strlen(argv[i]);
            if (argument_length >= PX4_TASK_MAX_ARGV_LENGTH) {
                qurt_mutex_unlock(&task_mutex);
                PX4_ERR("Argument %d is too long %d", i, argument_length);
                return -1;
            } else {
                strcpy(taskmap[task_index].argv_storage[i], argv[i]);
                taskmap[task_index].argv[i] = taskmap[task_index].argv_storage[i];
            }
        } else {
            // Must add NULL at end of argv
            taskmap[task_index].argv[i] = nullptr;
            break;
        }
    }

    // Entry pointer for this task
	taskmap[task_index].main_entry = main_entry;
    taskmap[task_index].arg_entry = arg_entry;
    taskmap[task_index].arg = arg;

    // Convert priority into Qurt priority. Qurt has low number as highest
    // priority. PX4 uses high number as highest priority.
    if ((priority > 255) || (priority < 0)) {
        qurt_mutex_unlock(&task_mutex);
        PX4_ERR("Invalid priority %d", priority);
        return -1;
    }
    priority = 255 - priority;

    // Don't let the priority get above what the Qurt critical tasks need
    if (priority < 5) priority = 5;

    // Likewise, don't let the priority get too low as it will be below Qurt
    // background tasks
    if (priority > 250) priority = 250;

    // Copy name into our storage and verify length
    if (strlen(name) >= PX4_TASK_MAX_NAME_LENGTH) {
        qurt_mutex_unlock(&task_mutex);
        PX4_ERR("Task name is too long %s", name);
        return -1;
    }
    strcpy(taskmap[task_index].name, "PX4_");
    strcpy(&taskmap[task_index].name[4], name);

    // Create the thread with desired attributes
    qurt_thread_attr_init(&taskmap[task_index].attr);
    qurt_thread_attr_set_name(&taskmap[task_index].attr, taskmap[task_index].name);
    qurt_thread_attr_set_stack_addr(&taskmap[task_index].attr, taskmap[task_index].stack);
    qurt_thread_attr_set_stack_size(&taskmap[task_index].attr, PX4_TASK_STACK_SIZE);
    qurt_thread_attr_set_priority(&taskmap[task_index].attr, priority);

    retcode = qurt_thread_create(&taskmap[task_index].tid, &taskmap[task_index].attr, entry_adapter, (void*) &taskmap[task_index]);
    if (retcode != QURT_EOK) {
        qurt_mutex_unlock(&task_mutex);
        PX4_ERR("Couldn't create qurt thread %s", name);
        return -1;
    } else {
        PX4_INFO("Successfully created px4 task %s with tid %u",
                 taskmap[task_index].name,
                 (unsigned int) taskmap[task_index].tid);
    }

	taskmap[task_index].isused = true;

    qurt_mutex_unlock(&task_mutex);

	return i;
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[]) {
    if (entry == nullptr) {
        PX4_ERR("Entry function pointer is null");
        return -1;
    }
    return px4_task_spawn_internal(name, priority, entry, argv,
                                   nullptr, nullptr);
}

px4_task_t px4_task_spawn(const char *name,
				       int priority,
				       px4_qurt_task_func_t entry,
				       void *arg) {
    if (entry == nullptr) {
        PX4_ERR("Entry function pointer is null");
        return -1;
    }
    return px4_task_spawn_internal(name, priority, nullptr, nullptr,
                                   entry, arg);
}

int px4_task_delete(px4_task_t id)
{
	int rv = 0;

    PX4_ERR("Ignoring px4_task_delete for task %d", id);

	// pthread_t pid;
	// PX4_WARN("Called px4_task_delete");
    //
	// if (id < PX4_MAX_TASKS && taskmap[id].isused) {
	// 	pid = taskmap[id].pid;
    //
	// } else {
	// 	return -EINVAL;
	// }
    //
	// // If current thread then exit, otherwise cancel
	// if (pthread_self() == pid) {
	// 	taskmap[id].isused = false;
	// 	pthread_exit(0);
    //
	// } else {
	// 	rv = pthread_cancel(pid);
	// }
    //
	// taskmap[id].isused = false;

	return rv;
}

void px4_task_exit(int ret)
{
    PX4_ERR("Ignoring px4_task_exit with return value %d", ret);

	// int i;
	// pthread_t pid = pthread_self();
    //
	// // Get pthread ID from the opaque ID
	// for (i = 0; i < PX4_MAX_TASKS; ++i) {
	// 	if (taskmap[i].pid == pid) {
	// 		taskmap[i].isused = false;
	// 		break;
	// 	}
	// }
    //
	// if (i >= PX4_MAX_TASKS)  {
	// 	PX4_ERR("px4_task_exit: self task not found!");
    //
	// } else {
	// 	PX4_DEBUG("px4_task_exit: %s", taskmap[i].name.c_str());
	// }
    //
	// pthread_exit((void *)(unsigned long)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
    // This is supposed to bring a thread out of it's sleep. But for Qurt
    // you cannot interrupt the sleep with a signal.
    // TODO: Come up with a different scheme
    PX4_DEBUG("Ignoring px4_task_kill with id %d and signal %d", id, sig);

	int rv = 0;
	// pthread_t pid;
	// PX4_DEBUG("Called px4_task_kill %d, taskname %s", sig, taskmap[id].name.c_str());
    //
	// if (id < PX4_MAX_TASKS && taskmap[id].pid != 0) {
	// 	pid = taskmap[id].pid;
    //
	// } else {
	// 	return -EINVAL;
	// }
    //
	// // If current thread then exit, otherwise cancel
	// rv = pthread_kill(pid, sig);

	return rv;
}

void px4_show_tasks()
{
	int idx = 0;
	int count = 0;

	PX4_INFO("Active Tasks:");

	for (; idx < PX4_MAX_TASKS; idx++) {
		if (taskmap[idx].isused) {
			PX4_INFO("   %-10s %u", taskmap[idx].name,
                     (unsigned int) taskmap[idx].tid);
			count++;
		}
	}

	if (count == 0) {
		PX4_INFO("   No running tasks");
	}
}

px4_task_t px4_getpid()
{
    qurt_thread_t tid = qurt_thread_get_id();

	// Get pthread ID from the opaque ID
	for (int i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].tid == tid) {
			return i;
		}
	}

	return ~0;
}


const char *px4_get_taskname()
{
    qurt_thread_t tid = qurt_thread_get_id();

	// Get pthread ID from the opaque ID
	for (int i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].tid == tid) {
			return taskmap[i].name;
		}
	}

	return "Unknown App";
}

static void timer_cb(void *data)
{
	px4_sem_t *sem = reinterpret_cast<px4_sem_t *>(data);

	sem_post(sem);
}

int px4_sem_timedwait(px4_sem_t *sem, const struct timespec *ts)
{
	work_s _hpwork = {};

	// Get the current time.
	struct timespec ts_now;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts_now);

	// We get an absolute time but want to calculate a timeout in us.
	hrt_abstime timeout_us = ts_to_abstime((struct timespec *)ts) - ts_to_abstime(&ts_now);

	// Create a timer to unblock.
	hrt_work_queue(&_hpwork, (worker_t)&timer_cb, (void *)sem, timeout_us);
	sem_wait(sem);
	hrt_work_cancel(&_hpwork);
	return 0;
}

int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	int rv = -1;

    PX4_ERR("Ignoring px4_prctl %d, %p, %d", option, arg2, pid);

	// switch (option) {
	// case PR_SET_NAME:
	// 	// set the threads name - Not supported
	// 	// rv = pthread_setname_np(pthread_self(), arg2);
	// 	rv = -1;
	// 	break;
    //
	// default:
	// 	rv = -1;
	// 	PX4_WARN("FAILED SETTING TASK NAME");
	// 	break;
	// }

	return rv;
}
