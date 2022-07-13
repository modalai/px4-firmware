/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file px4io_serial.cpp
 *
 * Serial interface for PX4IO on Posix platform
 */

#include <px4_arch/px4io_serial.h>
#include <drivers/device/qurt/uart.h>

#define ASYNC_UART_READ_WAIT_US 2000

uint8_t ArchPX4IOSerial::_io_buffer_storage[sizeof(IOPacket)];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_current_packet(nullptr)
{
	uart_fd = -1;
}

ArchPX4IOSerial::~ArchPX4IOSerial(){}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r;

	if ((r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0])) != 0) {
		return r;
	}

	if (uart_fd < 0) {
		// uart_fd = qurt_uart_open("2", 1000000);
		uart_fd = qurt_uart_open("2", 921600);
	}

	if (uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	} else {
		PX4_INFO("serial port fd %d", uart_fd);
	}

	return 0;
}

int
ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{

	PX4_INFO("%s called", __FUNCTION__);

	switch (operation) {

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			PX4_INFO("test 0\n");
			return 0;

		case 1:
			PX4_INFO("test 1\n");
			return 0;

		case 2:
			PX4_INFO("test 2\n");
			return 0;
		}

	default:
		break;
	}

	return -1;
}

int
ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	_current_packet = _packet;

	perf_begin(_pc_txns);

	int ret = qurt_uart_write(uart_fd, (const char*) _packet, sizeof(IOPacket));

	if (ret > 0) {
			// PX4_INFO("Write %d bytes", ret);

			usleep(20000);

		    // The UART read on SLPI is via an asynchronous service so specify a timeout
		    // for the return. The driver will poll periodically until the read comes in
		    // so this may block for a while. However, it will timeout if no read comes in.
		    ret = qurt_uart_read(uart_fd, (char*) _packet, sizeof(IOPacket), ASYNC_UART_READ_WAIT_US);

			if (ret > 0){
				// PX4_INFO("Read %d bytes", ret);

				/* Check CRC */
				uint8_t crc = _packet->crc;
				_packet->crc = 0;

				if ((crc != crc_packet(_packet)) || (PKT_CODE(*_packet) == PKT_CODE_CORRUPT)){
					perf_count(_pc_crcerrs);
					perf_end(_pc_txns);
					PX4_ERR("Packet CRC error");
					return -EIO;
				}
			}
	}

	if (ret <= 0) {
		// Not really a DMA failure, but we don't use DMA so we'll reuse the
		// counter to mean read / write failures.
		// perf_count(_pc_dmaerrs);
		perf_cancel(_pc_txns);		/* don't count this as a transaction */
		return -EIO;
	}

	perf_end(_pc_txns);
	return 0;
}
