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

#include <poll.h>
#include <termios.h>

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
		uart_fd = open("/dev/ttyHS1", O_RDWR | O_NONBLOCK);
	}

	if (uart_fd < 0) {
		PX4_ERR("Open failed in %s", __FUNCTION__);
		return -1;
	} else {
		PX4_INFO("serial port fd %d", uart_fd);
	}

	// Configuration copied from dsm_config
	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(uart_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
			                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, B921600)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, B921600)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
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

	struct pollfd fd_monitor;
	fd_monitor.fd     = uart_fd;
	fd_monitor.events = POLLIN;

	perf_begin(_pc_txns);

	tcflush(uart_fd, TCIOFLUSH);

	int ret = ::write(uart_fd, _packet, sizeof(IOPacket));

	if (ret > 0) {
		// PX4_INFO("Write %d bytes", ret);

		// int poll_status = ::poll(&fd_monitor, 1, 1000);
		(void) ::poll(&fd_monitor, 1, 1000);

		ret = ::read(uart_fd, _packet, sizeof(IOPacket));

		if (ret > 0){
			// PX4_INFO("Read %d bytes", ret);

			/* Check CRC */
			uint8_t crc = _packet->crc;
			_packet->crc = 0;

			if ((crc != crc_packet(_packet)) || (PKT_CODE(*_packet) == PKT_CODE_CORRUPT)){
				perf_count(_pc_crcerrs);
				perf_end(_pc_txns);
				// PX4_ERR("Packet CRC error");
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
