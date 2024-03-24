/****************************************************************************
 *
 *   Copyright (c) 2024 ModalAI, Inc. All rights reserved.
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
 * @file uart_test.cpp
 *
 */

#include <string.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <px4_log.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/qurt/uart.h>

#define UART_TEST_DEVICE_PATH "2"
#define ASYNC_UART_READ_WAIT_US 2000

#define UNUSED(x) (void)(x)

extern "C" { __EXPORT int uart_test_main(int argc, char *argv[]); }

namespace uart_test
{

volatile bool _task_should_exit = false;
static bool _is_running = false;
static px4_task_t _task_handle = -1;

int start();
int stop();
int info();
void usage();

void task_main(int argc, char *argv[])
{
	const char *device_path = UART_TEST_DEVICE_PATH;

	const int baudrate[] = {9600, 38400, 57600, 115200, 230400, 250000, 420000, 460800, 921600, 1843200, 2000000};
	// const int baudrate[] = {115200, 115200, 115200, 115200, 115200, 115200, 115200, 115200, 115200, 115200, 115200};
	// const int baudrate[] = {9600, 9600, 9600, 9600, 9600, 9600, 9600, 9600, 9600, 9600, 9600};
	const int num_baudrates = 11;
	int baudrate_index = 0;

	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			break;
		}
	}

	sleep(2);

	PX4_INFO("UART TEST: Setting baudrate to %d", baudrate[baudrate_index]);
	int uart_fd = qurt_uart_open(device_path, baudrate[baudrate_index]);

	if (uart_fd < 0) {
		PX4_ERR("UART TEST: qurt_uart_open failed");
		return;

	} else {
		PX4_INFO("UART TEST: qurt_uart_open succeeded");
	}

	uint8_t rx_buf[500];

	_is_running = true;

	sleep(1);

	// Main loop
	while (!_task_should_exit) {

		for (int i = 0; i < 10; i++) {
			// The UART read on SLPI is via an asynchronous service so specify a timeout
			// for the return. The driver will poll periodically until the read comes in
			// so this may block for a while. However, it will timeout if no read comes in.
			memset(rx_buf, 0, sizeof(rx_buf));
			int newbytes = qurt_uart_read(uart_fd, (char *) &rx_buf[0], sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);
			
			if (newbytes <= 0) {
				// PX4_INFO("UART TEST: Read no bytes from UART");
			
			} else {
				PX4_INFO("UART TEST: Read %d bytes from UART", newbytes);
			
				PX4_INFO("UART TEST [0-7]: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x",
					 rx_buf[0],
					 rx_buf[1],
					 rx_buf[2],
					 rx_buf[3],
					 rx_buf[4],
					 rx_buf[5],
					 rx_buf[6],
					 rx_buf[7]);
				if (newbytes > 8) {
					PX4_INFO("UART TEST [8-15]: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x",
						 rx_buf[8],
						 rx_buf[9],
						 rx_buf[10],
						 rx_buf[11],
						 rx_buf[12],
						 rx_buf[13],
						 rx_buf[14],
						 rx_buf[15]);
				}

				uint8_t write_data[5] = {0x23, 0x45, 0x09, 0x22, 0xfe};
				qurt_uart_write(uart_fd, (const char *) write_data, 5);
			}

			usleep(250000);
		}

		baudrate_index++;
		if (baudrate_index == num_baudrates) {
			baudrate_index = 0;
		}

		PX4_INFO("***");
		PX4_INFO("UART TEST: Setting baudrate to %d", baudrate[baudrate_index]);
		PX4_INFO("***");
		(void) qurt_uart_open(device_path, baudrate[baudrate_index]);

		usleep(250000);
	}

	_is_running = false;
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("uart_test_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

int info()
{
	PX4_INFO("running: %s", _is_running ? "yes" : "no");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: uart_test {start|info|stop}");
}

} // namespace uart_test


int uart_test_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		uart_test::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return uart_test::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return uart_test::stop();
	}

	else if (!strcmp(verb, "info")) {
		return uart_test::info();
	}

	else {
		uart_test::usage();
		return 1;
	}
}
