/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file virtual_serial_endpoint.cpp
 *
 * VOXL2 SLPI test bridge between a physical QURT UART and the virtual serial
 * uORB topics used by MAVLink SERIAL_CONTROL devices 100 through 109.
 */

#include <drivers/device/qurt/uart.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/uORB.h>
#include <uORB/topics/virtual_serial_receive.h>
#include <uORB/topics/virtual_serial_transmit.h>

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
__EXPORT int virtual_serial_endpoint_main(int argc, char *argv[]);
__EXPORT int fc_uart_rx_available(int fd, uint32_t *data);
}

namespace virtual_serial_endpoint
{

static constexpr uint8_t VSE_DEVICE_MIN = 100;
static constexpr uint8_t VSE_DEVICE_MAX = 109;
static constexpr size_t VSE_DATA_LEN = 70;
static constexpr int VSE_UART_READ_WAIT_US = 500;
static constexpr int VSE_TASK_STACK_SIZE = 2000;

static char _port[16]{};
static uint8_t _device_id{VSE_DEVICE_MIN};
static uint32_t _baudrate{115200};
static int _uart_fd{-1};
static int _tx_sub{-1};
static orb_advert_t _rx_pub{nullptr};
static px4_task_t _task_handle{-1};
static volatile bool _task_should_exit{false};
static volatile bool _tx_enabled{false};
static volatile bool _read_enabled{false};
static volatile bool _is_running{false};
static volatile bool _worker_active{false};

static uint16_t _rx_sequence{0};
static uint8_t _read_buf[VSE_DATA_LEN]{};

static uint32_t _virtual_tx_packets{0};
static uint32_t _virtual_tx_bytes{0};
static uint32_t _virtual_tx_control_packets{0};
static uint32_t _virtual_tx_ignored{0};
static uint32_t _uart_write_errors{0};
static uint32_t _uart_read_errors{0};
static uint32_t _uart_rx_packets{0};
static uint32_t _uart_rx_bytes{0};
static uint32_t _virtual_rx_publish_failures{0};

static bool ensure_tx_subscription()
{
	if (_tx_sub >= 0) {
		return true;
	}

	PX4_INFO("subscribing to virtual_serial_transmit");
	_tx_sub = orb_subscribe(ORB_ID(virtual_serial_transmit));

	if (_tx_sub < 0) {
		PX4_ERR("failed to subscribe to virtual_serial_transmit");
		return false;
	}

	PX4_INFO("subscribed to virtual_serial_transmit fd=%d", _tx_sub);
	return true;
}

static void print_uart_rx(const uint8_t *data, size_t len)
{
	char hex[24]{};
	static constexpr char nibble_to_hex[] = "0123456789abcdef";
	const size_t bytes_to_print = len < 8 ? len : 8;
	size_t offset = 0;

	for (size_t i = 0; i < bytes_to_print; i++) {
		if (i > 0) {
			hex[offset++] = ' ';
		}

		hex[offset++] = nibble_to_hex[(data[i] >> 4) & 0x0f];
		hex[offset++] = nibble_to_hex[data[i] & 0x0f];
	}

	PX4_INFO("UART RX %u bytes: %s", static_cast<unsigned>(len), hex);
}

static void process_virtual_tx(bool force = false)
{
	if (!_tx_enabled && !force) {
		return;
	}

	if (!ensure_tx_subscription()) {
		return;
	}

	if (_tx_sub < 0) {
		return;
	}

	virtual_serial_transmit_s msg{};
	int updates = 0;
	bool updated = false;

	while (updates++ < virtual_serial_transmit_s::ORB_QUEUE_LENGTH &&
	       orb_check(_tx_sub, &updated) == PX4_OK && updated) {

		if (orb_copy(ORB_ID(virtual_serial_transmit), _tx_sub, &msg) != PX4_OK) {
			break;
		}

		if (msg.device != _device_id) {
			_virtual_tx_ignored++;
			continue;
		}

		const size_t len = msg.data_length < VSE_DATA_LEN ? msg.data_length : VSE_DATA_LEN;

		if (len == 0) {
			_virtual_tx_control_packets++;
			continue;
		}

		const int written = qurt_uart_write(_uart_fd, reinterpret_cast<const char *>(msg.data), len);

		if (written < 0 || static_cast<size_t>(written) != len) {
			_uart_write_errors++;
			continue;
		}

		_virtual_tx_packets++;
		_virtual_tx_bytes += static_cast<uint32_t>(len);
	}
}

static void publish_virtual_rx(const uint8_t *data, size_t len)
{
	virtual_serial_receive_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.sequence = _rx_sequence++;
	msg.device = _device_id;
	msg.data_length = static_cast<uint8_t>(len < VSE_DATA_LEN ? len : VSE_DATA_LEN);

	if (msg.data_length > 0 && data != nullptr) {
		memcpy(msg.data, data, msg.data_length);
	}

	if (_rx_pub == nullptr) {
		_rx_pub = orb_advertise(ORB_ID(virtual_serial_receive), &msg);

		if (_rx_pub == nullptr) {
			_virtual_rx_publish_failures++;
			return;
		}

	} else if (orb_publish(ORB_ID(virtual_serial_receive), _rx_pub, &msg) != PX4_OK) {
		_virtual_rx_publish_failures++;
		return;
	}

	_uart_rx_packets++;
	_uart_rx_bytes += msg.data_length;
}

static void process_uart_rx()
{
	for (int reads = 0; reads < virtual_serial_receive_s::ORB_QUEUE_LENGTH; reads++) {
		uint32_t rx_bytes_available = 0;

		if (fc_uart_rx_available(_uart_fd, &rx_bytes_available) < 0) {
			_uart_read_errors++;
			return;
		}

		if (rx_bytes_available == 0) {
			return;
		}

		PX4_INFO("UART RX available %" PRIu32 " bytes", rx_bytes_available);
		const size_t read_len = rx_bytes_available < sizeof(_read_buf) ? rx_bytes_available : sizeof(_read_buf);
		const int nread = qurt_uart_read(_uart_fd, reinterpret_cast<char *>(_read_buf), read_len,
						 VSE_UART_READ_WAIT_US);

		if (nread < 0) {
			_uart_read_errors++;
			return;
		}

		if (nread == 0) {
			return;
		}

		print_uart_rx(_read_buf, static_cast<size_t>(nread));
		publish_virtual_rx(_read_buf, static_cast<size_t>(nread));
	}
}

static int task_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	PX4_INFO("virtual_serial_endpoint task running");
	_worker_active = true;

	while (!_task_should_exit) {
		if (_tx_enabled) {
			process_virtual_tx();
		}

		if (_read_enabled) {
			process_uart_rx();
		}

		usleep(1000);
	}

	_worker_active = false;

	if (_tx_sub >= 0) {
		orb_unsubscribe(_tx_sub);
		_tx_sub = -1;
	}

	_uart_fd = -1;
	_is_running = false;
	_task_handle = -1;
	PX4_INFO("virtual_serial_endpoint task exiting");
	return 0;
}

static int spawn_worker()
{
	if (_worker_active || _task_handle >= 0) {
		PX4_WARN("worker already started");
		return PX4_OK;
	}

	PX4_INFO("spawning worker");
	// QURT can fault on longer pthread names, so keep this task name short.
	_task_handle = px4_task_spawn_cmd("vse_worker",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  VSE_TASK_STACK_SIZE,
					  (px4_main_t)&task_main,
					  nullptr);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int start(int argc, char *argv[])
{
	const char *port = nullptr;
	long device_id = -1;
	uint32_t baudrate = 0;
	bool baudrate_set = false;
	bool start_worker = true;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:v:b:n", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			port = myoptarg;
			break;

		case 'v':
			device_id = strtol(myoptarg, nullptr, 0);
			break;

		case 'b':
			baudrate = strtoul(myoptarg, nullptr, 0);
			baudrate_set = true;
			break;

		case 'n':
			start_worker = false;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (_is_running) {
		PX4_WARN("already running");
		return PX4_OK;
	}

	if (port == nullptr) {
		PX4_ERR("missing UART port");
		return PX4_ERROR;
	}

	if (device_id < VSE_DEVICE_MIN || device_id > VSE_DEVICE_MAX) {
		PX4_ERR("virtual device must be 100..109");
		return PX4_ERROR;
	}

	if (!baudrate_set || baudrate == 0) {
		PX4_ERR("missing or invalid UART baudrate");
		return PX4_ERROR;
	}

	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
	_device_id = static_cast<uint8_t>(device_id);
	_baudrate = baudrate;
	_task_should_exit = false;
	_tx_enabled = true;
	_read_enabled = true;

	PX4_INFO("opening UART %s at %" PRIu32 " baud for virtual device %u",
		 _port, _baudrate, static_cast<unsigned>(_device_id));
	_uart_fd = qurt_uart_open(_port, _baudrate);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open UART port %s", _port);
		return PX4_ERROR;
	}

	_is_running = true;

	if (!start_worker) {
		PX4_INFO("worker not started");
		return PX4_OK;
	}

	if (spawn_worker() != PX4_OK) {
		_is_running = false;
		_uart_fd = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int worker()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	if (_uart_fd < 0) {
		PX4_ERR("UART is not open");
		return PX4_ERROR;
	}

	return spawn_worker();
}

static int stop()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	_task_should_exit = true;
	_tx_enabled = false;
	_read_enabled = false;

	if (_task_handle < 0) {
		_is_running = false;
		_uart_fd = -1;
	}

	return PX4_OK;
}

static int poll()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	if (_uart_fd < 0) {
		PX4_ERR("UART is not open");
		return PX4_ERROR;
	}

	PX4_INFO("polling UART once");
	process_uart_rx();
	return PX4_OK;
}

static int txpoll()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	if (_uart_fd < 0) {
		PX4_ERR("UART is not open");
		return PX4_ERROR;
	}

	PX4_INFO("polling virtual TX once");
	process_virtual_tx(true);
	return PX4_OK;
}

static int rxstart()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	_read_enabled = true;
	PX4_INFO("UART RX polling enabled");
	return PX4_OK;
}

static int txstart()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	if (!_worker_active) {
		PX4_ERR("worker is not active");
		return PX4_ERROR;
	}

	_tx_enabled = true;
	PX4_INFO("virtual TX polling enabled");
	return PX4_OK;
}

static int txstop()
{
	_tx_enabled = false;
	PX4_INFO("virtual TX polling disabled");
	return PX4_OK;
}

static int rxstop()
{
	_read_enabled = false;
	PX4_INFO("UART RX polling disabled");
	return PX4_OK;
}

static int status()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	PX4_INFO("UART port=%s baud=%" PRIu32 " virtual_device=%u open=%s",
		 _port, _baudrate, static_cast<unsigned>(_device_id),
		 _uart_fd >= 0 ? "yes" : "no");
	PX4_INFO("worker=%s virtual_tx_polling=%s",
		 _worker_active ? "active" : "inactive", _tx_enabled ? "enabled" : "disabled");
	PX4_INFO("uart_rx_polling=%s", _read_enabled ? "enabled" : "disabled");
	PX4_INFO("virtual->uart packets=%" PRIu32 " bytes=%" PRIu32 " control=%" PRIu32 " ignored=%" PRIu32,
		 _virtual_tx_packets, _virtual_tx_bytes, _virtual_tx_control_packets, _virtual_tx_ignored);
	PX4_INFO("uart->virtual packets=%" PRIu32 " bytes=%" PRIu32 " publish_fail=%" PRIu32,
		 _uart_rx_packets, _uart_rx_bytes, _virtual_rx_publish_failures);
	PX4_INFO("uart_write_errors=%" PRIu32 " uart_read_errors=%" PRIu32, _uart_write_errors, _uart_read_errors);
	return PX4_OK;
}

static int usage()
{
	PX4_INFO("Usage: virtual_serial_endpoint start -p <uart> -v <100..109> -b <baudrate> [-n]");
	PX4_INFO("       virtual_serial_endpoint status");
	PX4_INFO("       virtual_serial_endpoint poll");
	PX4_INFO("       virtual_serial_endpoint txpoll");
	PX4_INFO("       virtual_serial_endpoint worker");
	PX4_INFO("       virtual_serial_endpoint txstart");
	PX4_INFO("       virtual_serial_endpoint txstop");
	PX4_INFO("       virtual_serial_endpoint rxstart");
	PX4_INFO("       virtual_serial_endpoint rxstop");
	PX4_INFO("       virtual_serial_endpoint stop");
	return PX4_OK;
}

} // namespace virtual_serial_endpoint

int virtual_serial_endpoint_main(int argc, char *argv[])
{
	if (argc <= 1) {
		return virtual_serial_endpoint::usage();
	}

	if (!strcmp(argv[1], "start")) {
		return virtual_serial_endpoint::start(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "stop")) {
		return virtual_serial_endpoint::stop();
	}

	if (!strcmp(argv[1], "status")) {
		return virtual_serial_endpoint::status();
	}

	if (!strcmp(argv[1], "poll")) {
		return virtual_serial_endpoint::poll();
	}

	if (!strcmp(argv[1], "txpoll")) {
		return virtual_serial_endpoint::txpoll();
	}

	if (!strcmp(argv[1], "worker")) {
		return virtual_serial_endpoint::worker();
	}

	if (!strcmp(argv[1], "rxstart")) {
		return virtual_serial_endpoint::rxstart();
	}

	if (!strcmp(argv[1], "txstart")) {
		return virtual_serial_endpoint::txstart();
	}

	if (!strcmp(argv[1], "txstop")) {
		return virtual_serial_endpoint::txstop();
	}

	if (!strcmp(argv[1], "rxstop")) {
		return virtual_serial_endpoint::rxstop();
	}

	return virtual_serial_endpoint::usage();
}
