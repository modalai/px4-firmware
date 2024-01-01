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

#include <poll.h>
#include <termios.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/qurt/uart.h>
#include <px4_platform_common/px4_serial.h>

#define PX4_SERIAL_MAX_RX_DATA 4096
#define PX4_SERIAL_MAX_PORTS 3

struct px4_serial_data_t {
	char port[2];
	bool open;
	uint32_t speed;
	uint32_t poll_bytes_received;
	uint8_t receive_data[PX4_SERIAL_MAX_RX_DATA];
};

static struct px4_serial_data_t px4_serial_data[PX4_SERIAL_MAX_PORTS];

int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	int fd = -1;

	if (fds == NULL) {
		return -1;
	}

	if (nfds != 1) {
		// Only support a single serial port at a time
		return -1;
	}

	fd = fds[0].fd;

	if (fd < 0) {
		return -1;
	}

	if (fd >= PX4_SERIAL_MAX_PORTS) {
		return -1;
	}

	if (px4_serial_data[fd].poll_bytes_received) {
		fds[0].revents |= POLLIN;
		return px4_serial_data[fd].poll_bytes_received;
	}

	hrt_abstime now = hrt_absolute_time();
	int bytes_read = 0;

	while (hrt_elapsed_time(&now) < (hrt_abstime)(timeout * 1000)) {
		bytes_read = px4_serial_read(fd, (void*) px4_serial_data[fd].receive_data,
									 PX4_SERIAL_MAX_RX_DATA);

		if (bytes_read > 0) {
			px4_serial_data[fd].poll_bytes_received += bytes_read;
			fds[0].revents |= POLLIN;
			break;
		}
	}

	return bytes_read;

	return -1;
}

int tcgetattr(int fd, struct termios *termios_p)
{
	// Qurt doesn't use the attributes so no need to return anything
	return 0;
}

int cfsetspeed(struct termios *termios_p, speed_t speed)
{
	if (termios_p == NULL) {
		return -1;
	}

	// Not all of these are actually supported but we'll let the
	// Qurt uart driver figure that out.
	switch (speed) {
	case B9600:
		termios_p->c_speed = 9600;
		break;
	case B19200:
		termios_p->c_speed = 19200;
		break;
	case B38400:
		termios_p->c_speed = 38400;
		break;
	case B57600:
		termios_p->c_speed = 57600;
		break;
	case B115200:
		termios_p->c_speed = 115200;
		break;
	case B230400:
		termios_p->c_speed = 230400;
		break;
	case B460800:
		termios_p->c_speed = 460800;
		break;
	case B500000:
		termios_p->c_speed = 500000;
		break;
	case B576000:
		termios_p->c_speed = 576000;
		break;
	case B921600:
		termios_p->c_speed = 921600;
		break;
	case B1000000:
		termios_p->c_speed = 1000000;
		break;
	case B1152000:
		termios_p->c_speed = 1152000;
		break;
	case B1500000:
		termios_p->c_speed = 1500000;
		break;
	case B2000000:
		termios_p->c_speed = 2000000;
		break;
	case B2500000:
		termios_p->c_speed = 2500000;
		break;
	case B3000000:
		termios_p->c_speed = 3000000;
		break;
	// This is not POSIX compliant but is NuttX compliant
	// and so needs to be supported here
	case 420000:
		termios_p->c_speed = speed;
		break;
	default:
		return -1;
	}

	return 0;
}

int cfsetispeed(struct termios *termios_p, speed_t speed)
{
	return cfsetspeed(termios_p, speed);
}

int cfsetospeed(struct termios *termios_p, speed_t speed)
{
	return cfsetspeed(termios_p, speed);
}

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	if (termios_p == NULL) {
		return -1;
	}

	if (fd < 0) {
		return -1;
	}

	if (fd >= PX4_SERIAL_MAX_PORTS) {
		return -1;
	}

	// This is where the UART is really setup and configured in Qurt
	return qurt_uart_open(px4_serial_data[fd].port, termios_p->c_speed);
}

// TODO
// int tcflush(int fd, int queue_selector);

int px4_serial_access(const char *pathname, int mode)
{
	// Just blindly say that the path exists. If it isn't valid
	// then the open call will fail.
	return 0;
}

int px4_serial_open(const char *pathname, int flags)
{
	if (pathname == NULL) {
		return -1;
	}

	// There is a very specific mapping of pathname to
	// actual file descriptor. This is done in the qurt uart
	// driver. We don't actually want to set the driver up yet
	// so we have to do the mapping ourselves. This needs to match
	// what the qurt driver does or else everything will break!
	uint32_t port_number = strtol(pathname, NULL, 10);
	int temp_fd = -1;

	switch (port_number) {
        case 2:
			temp_fd = 0;
			strcpy(px4_serial_data[temp_fd].port, "2");
            break;
        case 6:
			temp_fd = 1;
			strcpy(px4_serial_data[temp_fd].port, "6");
            break;
        case 7:
			temp_fd = 2;
			strcpy(px4_serial_data[temp_fd].port, "7");
            break;
		default:
			return -1;
	}

	px4_serial_data[temp_fd].poll_bytes_received = 0;
	px4_serial_data[temp_fd].open = true;

	return temp_fd;
}

int px4_serial_close(int fd)
{
	if (fd < 0) {
		return -1;
	}

	if (fd >= PX4_SERIAL_MAX_PORTS) {
		return -1;
	}

	px4_serial_data[fd].open = false;
	px4_serial_data[fd].poll_bytes_received = 0;
	
	// Note: Qurt doesn't have a close method

	return 0;
}

ssize_t px4_serial_read(int fd, void *buf, size_t count)
{
	if (buf == NULL) {
		return -1;
	}

	if (fd < 0) {
		return -1;
	}

	if (fd >= PX4_SERIAL_MAX_PORTS) {
		return -1;
	}

	uint32_t bytes_to_copy = px4_serial_data[fd].poll_bytes_received;

	if (bytes_to_copy) {
		if (bytes_to_copy > count) {
			bytes_to_copy = count;
		}

		memcpy(buf, px4_serial_data[fd].receive_data, bytes_to_copy);

		px4_serial_data[fd].poll_bytes_received -= bytes_to_copy;

		// Make sure not to drop any data
		if (px4_serial_data[fd].poll_bytes_received) {
			memcpy(px4_serial_data[fd].receive_data,
				   &px4_serial_data[fd].receive_data[bytes_to_copy],
				   px4_serial_data[fd].poll_bytes_received);
		}

		return (ssize_t) bytes_to_copy;
	}

	return qurt_uart_read(fd, (char*) buf, count, 2000);
}

ssize_t px4_serial_write(int fd, const void *buf, size_t count)
{
	return qurt_uart_write(fd, buf, count);
}
