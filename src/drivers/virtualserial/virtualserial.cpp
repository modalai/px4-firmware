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
 * @file virtualserial.cpp
 *
 * Virtual serial bridge driven by MAVLink SERIAL_CONTROL messages.
 *
 * Data flow:
 *   SERIAL_CONTROL device 100..109 -> VirtualSerialTransmit uORB topic
 *   VirtualSerialReceive uORB topic -> SERIAL_CONTROL (FLAG_REPLY)
 */

#include "virtualserial.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/virtual_serial_receive.h>
#include <uORB/topics/virtual_serial_transmit.h>

#include <inttypes.h>
#include <pthread.h>
#include <string.h>

static constexpr uint8_t VS_DEVICE_MIN = 100;
static constexpr uint8_t VS_DEVICE_MAX = 109;
static constexpr size_t VS_DEVICE_COUNT = VS_DEVICE_MAX - VS_DEVICE_MIN + 1;
static constexpr size_t VS_DATA_LEN = 70;
static constexpr size_t VS_RX_BUF_SIZE = 1024;

class VirtualSerialManager
{
private:
	struct PortState {
		bool route_valid{false};
		uint8_t target_sysid{0};
		uint8_t target_compid{0};
		uint8_t channel{255};
		uint16_t tx_sequence{0};

		uint8_t rx_buf[VS_RX_BUF_SIZE] {};
		size_t rx_len{0};

		uint32_t tx_published{0};
		uint32_t tx_bytes{0};
		uint32_t tx_publish_failed{0};
		uint32_t tx_truncated{0};
		uint32_t rx_forwarded{0};
		uint32_t rx_bytes{0};
		uint32_t rx_dropped_no_route{0};
		uint32_t rx_dropped_overflow{0};
	};

public:
	VirtualSerialManager()
	{
		pthread_mutex_init(&_mutex, nullptr);
	}

	~VirtualSerialManager()
	{
		pthread_mutex_destroy(&_mutex);
	}

	static bool is_valid_device(uint8_t device)
	{
		return device >= VS_DEVICE_MIN && device <= VS_DEVICE_MAX;
	}

	int push_from_mavlink(const uint8_t *data, size_t len,
			      uint32_t baudrate, uint16_t timeout, uint8_t flags,
			      uint8_t sysid, uint8_t compid, uint8_t device,
			      uint8_t channel)
	{
		if (!is_valid_device(device)) {
			return -1;
		}

		virtual_serial_transmit_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.baudrate = baudrate;
		msg.timeout = timeout;
		msg.device = device;
		msg.flags = flags;

		const size_t data_len = len < VS_DATA_LEN ? len : VS_DATA_LEN;
		msg.data_length = static_cast<uint8_t>(data_len);

		if (data != nullptr && data_len > 0) {
			memcpy(msg.data, data, data_len);
		}

		pthread_mutex_lock(&_mutex);

		drain_receive_locked();

		PortState &port = _ports[index_for_device(device)];

		port.route_valid = true;
		port.target_sysid = sysid;
		port.target_compid = compid;
		port.channel = channel;
		msg.sequence = port.tx_sequence++;

		if (len > data_len) {
			port.tx_truncated++;
		}

		const bool published = _tx_pub.publish(msg);

		if (published) {
			port.tx_published++;
			port.tx_bytes += static_cast<uint32_t>(data_len);

		} else {
			port.tx_publish_failed++;
		}

		pthread_mutex_unlock(&_mutex);
		return published ? 0 : -1;
	}

	size_t pop_to_mavlink(uint8_t *buf, size_t max_len,
			      uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
			      uint8_t channel)
	{
		if (buf == nullptr || max_len == 0 ||
		    out_sysid == nullptr || out_compid == nullptr || out_device == nullptr) {
			return 0;
		}

		pthread_mutex_lock(&_mutex);

		drain_receive_locked();

		for (size_t i = 0; i < VS_DEVICE_COUNT; i++) {
			PortState &port = _ports[i];

			if (!port.route_valid || port.channel != channel || port.rx_len == 0) {
				continue;
			}

			const size_t to_copy = port.rx_len < max_len ? port.rx_len : max_len;
			memcpy(buf, port.rx_buf, to_copy);
			port.rx_len -= to_copy;

			if (port.rx_len > 0) {
				memmove(port.rx_buf, port.rx_buf + to_copy, port.rx_len);
			}

			*out_sysid = port.target_sysid;
			*out_compid = port.target_compid;
			*out_device = static_cast<uint8_t>(VS_DEVICE_MIN + i);
			port.rx_forwarded++;
			port.rx_bytes += static_cast<uint32_t>(to_copy);

			pthread_mutex_unlock(&_mutex);
			return to_copy;
		}

		pthread_mutex_unlock(&_mutex);
		return 0;
	}

	int print_status()
	{
		pthread_mutex_lock(&_mutex);

		int active_count = 0;

		for (size_t i = 0; i < VS_DEVICE_COUNT; i++) {
			const PortState &port = _ports[i];

			if (!port.route_valid && port.tx_published == 0 && port.tx_publish_failed == 0 &&
			    port.rx_forwarded == 0 && port.rx_dropped_no_route == 0 &&
			    port.rx_dropped_overflow == 0 && port.rx_len == 0) {
				continue;
			}

			PX4_INFO("device=%u route=%s sysid=%u compid=%u chan=%u pending=%zu",
				 static_cast<unsigned>(VS_DEVICE_MIN + i),
				 port.route_valid ? "yes" : "no",
				 static_cast<unsigned>(port.target_sysid),
				 static_cast<unsigned>(port.target_compid),
				 static_cast<unsigned>(port.channel),
				 port.rx_len);
			PX4_INFO("  tx_pub=%" PRIu32 " tx_bytes=%" PRIu32 " tx_fail=%" PRIu32 " tx_trunc=%" PRIu32,
				 port.tx_published, port.tx_bytes, port.tx_publish_failed, port.tx_truncated);
			PX4_INFO("  rx_fwd=%" PRIu32 " rx_bytes=%" PRIu32 " drop_no_route=%" PRIu32 " drop_overflow=%" PRIu32,
				 port.rx_forwarded, port.rx_bytes, port.rx_dropped_no_route, port.rx_dropped_overflow);
			active_count++;
		}

		if (active_count == 0) {
			PX4_INFO("virtualserial: no activity");
		}

		if (_rx_dropped_bad_device > 0) {
			PX4_INFO("rx_dropped_bad_device=%" PRIu32, _rx_dropped_bad_device);
		}

		pthread_mutex_unlock(&_mutex);
		return 0;
	}

private:
	static size_t index_for_device(uint8_t device)
	{
		return static_cast<size_t>(device - VS_DEVICE_MIN);
	}

	void drain_receive_locked()
	{
		virtual_serial_receive_s msg{};
		int updates = 0;

		while (updates++ < virtual_serial_receive_s::ORB_QUEUE_LENGTH && _rx_sub.update(&msg)) {
			if (!is_valid_device(msg.device)) {
				_rx_dropped_bad_device++;
				continue;
			}

			PortState &port = _ports[index_for_device(msg.device)];
			const size_t data_len = msg.data_length < VS_DATA_LEN ? msg.data_length : VS_DATA_LEN;

			if (data_len == 0) {
				continue;
			}

			if (!port.route_valid) {
				port.rx_dropped_no_route++;
				continue;
			}

			const size_t space = VS_RX_BUF_SIZE - port.rx_len;
			const size_t to_copy = data_len < space ? data_len : space;

			if (to_copy > 0) {
				memcpy(port.rx_buf + port.rx_len, msg.data, to_copy);
				port.rx_len += to_copy;
			}

			if (to_copy < data_len) {
				port.rx_dropped_overflow++;
			}
		}
	}

	pthread_mutex_t _mutex;
	uORB::Publication<virtual_serial_transmit_s> _tx_pub{ORB_ID(virtual_serial_transmit)};
	uORB::Subscription _rx_sub{ORB_ID(virtual_serial_receive)};
	PortState _ports[VS_DEVICE_COUNT] {};
	uint32_t _rx_dropped_bad_device{0};
};

static VirtualSerialManager &manager()
{
	static VirtualSerialManager instance;
	return instance;
}

bool VirtualSerial::isVirtualSerialDevice(uint8_t device)
{
	return VirtualSerialManager::is_valid_device(device);
}

int VirtualSerial::pushFromMavlink(const uint8_t *data, size_t len,
				   uint32_t baudrate, uint16_t timeout, uint8_t flags,
				   uint8_t sysid, uint8_t compid, uint8_t device,
				   uint8_t channel)
{
	return manager().push_from_mavlink(data, len, baudrate, timeout, flags, sysid, compid, device, channel);
}

size_t VirtualSerial::popToMavlink(uint8_t *buf, size_t max_len,
				   uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
				   uint8_t channel)
{
	return manager().pop_to_mavlink(buf, max_len, out_sysid, out_compid, out_device, channel);
}

int VirtualSerial::print_status()
{
	return manager().print_status();
}

int VirtualSerial::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Virtual serial bridge for MAVLink SERIAL_CONTROL devices 100 through 109.
Incoming SERIAL_CONTROL packets are published as VirtualSerialTransmit.
VirtualSerialReceive packets are returned as SERIAL_CONTROL replies.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("virtualserial", "driver");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

extern "C" __EXPORT int virtualserial_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "status") == 0) {
		return VirtualSerial::print_status();
	}

	return VirtualSerial::print_usage(argc < 2 ? nullptr : "unknown command");
}
