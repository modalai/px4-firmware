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
 * @file virtualserial.hpp
 *
 * MAVLink SERIAL_CONTROL virtual device bridge.
 */

#pragma once

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/virtual_serial_receive.h>
#include <uORB/topics/virtual_serial_transmit.h>

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t VS_DEVICE_MIN = 100;
static constexpr uint8_t VS_DEVICE_MAX = 109;
static constexpr size_t VS_DEVICE_COUNT = VS_DEVICE_MAX - VS_DEVICE_MIN + 1;
static constexpr size_t VS_DATA_LEN = 70;
static constexpr size_t VS_RX_BUF_SIZE = 1024;

class VirtualSerial
{
public:
	static bool isVirtualSerialDevice(uint8_t device);

	static int pushFromMavlink(const uint8_t *data, size_t len,
				   uint32_t baudrate, uint16_t timeout, uint8_t flags,
				   uint8_t sysid, uint8_t compid, uint8_t device,
				   uint8_t channel);

	static size_t popToMavlink(uint8_t *buf, size_t max_len,
				   uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
				   uint8_t channel);

	static int print_status();
	static int print_usage(const char *reason = nullptr);

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

	VirtualSerial();
	~VirtualSerial();

	static VirtualSerial &instance();
	static bool is_valid_device(uint8_t device);
	static size_t index_for_device(uint8_t device);

	int push_from_mavlink(const uint8_t *data, size_t len,
			      uint32_t baudrate, uint16_t timeout, uint8_t flags,
			      uint8_t sysid, uint8_t compid, uint8_t device,
			      uint8_t channel);

	size_t pop_to_mavlink(uint8_t *buf, size_t max_len,
			      uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
			      uint8_t channel);

	int print_status_impl();

	void drain_receive_locked();

	pthread_mutex_t _mutex;
	uORB::Publication<virtual_serial_transmit_s> _tx_pub{ORB_ID(virtual_serial_transmit)};
	uORB::Subscription _rx_sub{ORB_ID(virtual_serial_receive)};
	PortState _ports[VS_DEVICE_COUNT] {};
	uint32_t _rx_dropped_bad_device{0};
};
