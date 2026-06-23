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
 * VOXL2 SLPI bridge between a physical QURT UART and the virtual serial
 * uORB topics used by MAVLink SERIAL_CONTROL devices 100 through 109.
 */

#include <drivers/device/qurt/uart.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/tasks.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/virtual_serial_receive.h>
#include <uORB/topics/virtual_serial_transmit.h>

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

static char _port[16] {};
static uint8_t _device_id{VSE_DEVICE_MIN};
static uint32_t _baudrate{115200};
static int _uart_fd{-1};
static orb_advert_t _rx_pub{nullptr};
static px4_task_t _task_handle{-1};
static px4_sem_t _uart_lock;
static bool _uart_lock_initialized{false};
static volatile bool _task_should_exit{false};
static volatile bool _is_running{false};

static uint16_t _rx_sequence{0};
static uint8_t _read_buf[VSE_DATA_LEN] {};

struct Statistics {
	uint32_t virtual_tx_packets{};
	uint32_t virtual_tx_bytes{};
	uint32_t uart_write_errors{};
	uint32_t uart_read_errors{};
	uint32_t uart_rx_packets{};
	uint32_t uart_rx_bytes{};
	uint32_t virtual_rx_publish_failures{};
};

static Statistics _stats{};

static void reset_statistics()
{
	_stats = {};
	_rx_sequence = 0;
}

static bool init_uart_lock()
{
	if (_uart_lock_initialized) {
		return true;
	}

	if (px4_sem_init(&_uart_lock, 0, 1) != 0) {
		PX4_ERR("failed to initialize UART lock");
		return false;
	}

	_uart_lock_initialized = true;
	return true;
}

static void lock_uart()
{
	while (px4_sem_wait(&_uart_lock) != 0) {}
}

static void unlock_uart()
{
	px4_sem_post(&_uart_lock);
}

class TxWorkItem : public px4::ScheduledWorkItem
{
public:
	TxWorkItem() :
		ScheduledWorkItem("vse_tx", px4::wq_configurations::hp_default),
		_tx_sub(this, ORB_ID(virtual_serial_transmit))
	{
	}

	bool start()
	{
		_tx_sub.unsubscribe();
		_enabled = true;

		if (!_tx_sub.registerCallback()) {
			_enabled = false;
			return false;
		}

		return true;
	}

	void stop()
	{
		_enabled = false;
		_tx_sub.unregisterCallback();
		ScheduleClear();

		while (_running) {
			usleep(1000);
		}

		_tx_sub.unsubscribe();
	}

	bool registered() const { return _tx_sub.registered(); }

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _tx_sub;
	volatile bool _enabled{false};
	volatile bool _running{false};
};

static TxWorkItem &tx_work_item()
{
	static TxWorkItem item;
	return item;
}

static void process_virtual_tx(uORB::SubscriptionCallbackWorkItem &tx_sub)
{
	if (!_is_running || _uart_fd < 0) {
		return;
	}

	virtual_serial_transmit_s msg{};
	int updates = 0;

	while (updates++ < virtual_serial_transmit_s::ORB_QUEUE_LENGTH && tx_sub.update(&msg)) {
		if (msg.device != _device_id) {
			continue;
		}

		const size_t len = msg.data_length < VSE_DATA_LEN ? msg.data_length : VSE_DATA_LEN;

		if (len == 0) {
			continue;
		}

		lock_uart();
		const int written = qurt_uart_write(_uart_fd, reinterpret_cast<const char *>(msg.data), len);
		unlock_uart();

		if (written < 0 || static_cast<size_t>(written) != len) {
			_stats.uart_write_errors++;
			continue;
		}

		_stats.virtual_tx_packets++;
		_stats.virtual_tx_bytes += static_cast<uint32_t>(len);
	}
}

void TxWorkItem::Run()
{
	if (!_enabled) {
		return;
	}

	_running = true;

	if (_enabled) {
		process_virtual_tx(_tx_sub);
	}

	_running = false;
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
			_stats.virtual_rx_publish_failures++;
			return;
		}

	} else if (orb_publish(ORB_ID(virtual_serial_receive), _rx_pub, &msg) != PX4_OK) {
		_stats.virtual_rx_publish_failures++;
		return;
	}

	_stats.uart_rx_packets++;
	_stats.uart_rx_bytes += msg.data_length;
}

static void process_uart_rx()
{
	for (int reads = 0; reads < virtual_serial_receive_s::ORB_QUEUE_LENGTH; reads++) {
		uint32_t rx_bytes_available = 0;

		lock_uart();

		if (fc_uart_rx_available(_uart_fd, &rx_bytes_available) < 0) {
			unlock_uart();
			_stats.uart_read_errors++;
			return;
		}

		if (rx_bytes_available == 0) {
			unlock_uart();
			return;
		}

		const size_t read_len = rx_bytes_available < sizeof(_read_buf) ? rx_bytes_available : sizeof(_read_buf);
		const int nread = qurt_uart_read(_uart_fd, reinterpret_cast<char *>(_read_buf), read_len,
						 VSE_UART_READ_WAIT_US);
		unlock_uart();

		if (nread < 0) {
			_stats.uart_read_errors++;
			return;
		}

		if (nread == 0) {
			return;
		}

		publish_virtual_rx(_read_buf, static_cast<size_t>(nread));
	}
}

static int task_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	PX4_INFO("virtual_serial_endpoint task running");

	while (!_task_should_exit) {
		process_uart_rx();
		usleep(1000);
	}

	if (_rx_pub != nullptr) {
		orb_unadvertise(_rx_pub);
		_rx_pub = nullptr;
	}

	_uart_fd = -1;
	_is_running = false;
	_task_handle = -1;
	PX4_INFO("virtual_serial_endpoint task exiting");
	return 0;
}

static int spawn_worker()
{
	if (_task_handle >= 0) {
		PX4_WARN("worker already started");
		return PX4_OK;
	}

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
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:v:b:", &myoptind, &myoptarg)) != EOF) {
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

	if (!init_uart_lock()) {
		return PX4_ERROR;
	}

	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
	_device_id = static_cast<uint8_t>(device_id);
	_baudrate = baudrate;
	_task_should_exit = false;
	reset_statistics();

	PX4_INFO("opening UART %s at %" PRIu32 " baud for virtual device %u",
		 _port, _baudrate, static_cast<unsigned>(_device_id));
	_uart_fd = qurt_uart_open(_port, _baudrate);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open UART port %s", _port);
		return PX4_ERROR;
	}

	_is_running = true;

	if (!tx_work_item().start()) {
		PX4_ERR("failed to register virtual serial TX callback");
		_is_running = false;
		_uart_fd = -1;
		return PX4_ERROR;
	}

	if (spawn_worker() != PX4_OK) {
		tx_work_item().stop();
		_is_running = false;
		_uart_fd = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int stop()
{
	if (!_is_running) {
		PX4_INFO("not running");
		return PX4_OK;
	}

	_task_should_exit = true;
	tx_work_item().stop();

	while (_is_running) {
		usleep(200000);
	}

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
	PX4_INFO("rx_worker=%s tx_callback=%s",
		 _task_handle >= 0 ? "active" : "inactive",
		 tx_work_item().registered() ? "registered" : "unregistered");
	PX4_INFO("virtual->uart packets=%" PRIu32 " bytes=%" PRIu32 " write_errors=%" PRIu32,
		 _stats.virtual_tx_packets, _stats.virtual_tx_bytes, _stats.uart_write_errors);
	PX4_INFO("uart->virtual packets=%" PRIu32 " bytes=%" PRIu32 " publish_fail=%" PRIu32,
		 _stats.uart_rx_packets, _stats.uart_rx_bytes, _stats.virtual_rx_publish_failures);
	PX4_INFO("uart_read_errors=%" PRIu32, _stats.uart_read_errors);
	return PX4_OK;
}

static int usage()
{
	PX4_INFO("Usage: virtual_serial_endpoint start -p <uart> -v <100..109> -b <baudrate>");
	PX4_INFO("       virtual_serial_endpoint status");
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

	return virtual_serial_endpoint::usage();
}
