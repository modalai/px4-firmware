/****************************************************************************
 *
 *   Copyright (c) 2023 ModalAI, Inc. All rights reserved.
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
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/device/qurt/uart.h>

static px4_task_t   uart_poll_thread_tid;
static const char  *uart_poll_thread_name = "uart_poll_thread";

static int _uart_fd = -1;

extern "C" { __EXPORT int uart_poll_main(int argc, char *argv[]); }

#define ASYNC_UART_READ_WAIT_US 2000

static void uart_read_data() {
	uint8_t buf[1024];
	int bytes_read = 0;

	bytes_read = qurt_uart_read(_uart_fd, (char *) buf, 1024, ASYNC_UART_READ_WAIT_US);
	PX4_INFO("Read %d bytes from UART", bytes_read);
	if (bytes_read) {
		for (int i = 0; i < bytes_read; i++) {
			PX4_INFO("%d\t%u", i, buf[i]);
		}
	}
}

static void uart_poll_callback(int fd) {
	PX4_INFO("Got uart poll callback!");
	uart_read_data();
}

static int uart_poll_thread(int argc, char *argv[])
{
	PX4_INFO("Starting uart poll THREAD");

	if (_uart_fd >= 0) {
		PX4_ERR("Port already in use");
		return -1;
	}

	_uart_fd = qurt_uart_open("2", 115200);

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port 2");
		return -1;

	} else { PX4_INFO("qurt uart opened successfully"); }

	while (true) {
		usleep(1000000);
		int status = qurt_uart_poll(_uart_fd, uart_poll_callback, 0);
		if (status == 0) {
			PX4_INFO("uart poll callback failed, already data to read");
			uart_read_data();
		} else if (status == 1) {
			PX4_INFO("Successfully installed callback with qurt_uart_poll");
		} else {
			PX4_INFO("Got error from qurt_uart_poll");
		}
		PX4_INFO("UART thread polling");

	}

	return 0;
}

int uart_poll_main(int argc, char *argv[])
{
	uart_poll_thread_tid = px4_task_spawn_cmd(uart_poll_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     uart_poll_thread,
					     NULL);

	return 0;
}
