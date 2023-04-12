
#include <SerialImpl.hpp>
#include <string.h> // strncpy
#include <px4_log.h>
#include <drivers/device/qurt/uart.h>


namespace device
{

SerialImpl::SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		       FlowControl flowcontrol) :
	_baudrate(baudrate),
	_bytesize(bytesize),
	_parity(parity),
	_stopbits(stopbits),
	_flowcontrol(flowcontrol)
{
	if (port) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	}
}

SerialImpl::~SerialImpl()
{
	if (isOpen()) {
		close();
	}
}

bool SerialImpl::open()
{
	if (isOpen()) {
		return true;
	}

	int serial_fd = qurt_uart_open(_port, _baudrate);

	if (serial_fd < 0) {
		PX4_ERR("failed to open %s, errno: %d, %s", _port, errno, strerror(errno));
		return false;
	}

	_serial_fd = serial_fd;
	_open = true;

	if (configure()) {
		_configured = true;
	}

	return _open;
}

bool SerialImpl::isOpen() const
{
	return _open;
}

bool SerialImpl::close()
{
	int ret = 0;

	if (_serial_fd >= 0) {
		ret = qurt_uart_close(_serial_fd);
	}

	_serial_fd = -1;
	_open = false;
	_configured = false;

	return (ret == 0);
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
	if (!_open) {
		open();
	}

	if (!_configured) {
		configure();
	}

	int ret_read = qurt_uart_read(_serial_fd, (char*) buffer, buffer_size, 500);

	if (ret_read < 0) {
		PX4_DEBUG("%s read error %d", _port, ret_read);

	} else {
		_bytes_read += ret_read;
	}

	return ret_read;
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	// const size_t required_bytes = math::min(buffer_size, character_count);
	// const hrt_abstime start_time_us = hrt_absolute_time();
	// 
	// if (!_open) {
	// 	open();
	// }
	// 
	// if (!_configured) {
	// 	configure();
	// }
	// 
	// while (true) {
	// 	int bytes_available = 0;
	// 	int ioctl_ret = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);
	// 
	// 	if ((ioctl_ret == 0) && (bytes_available >= (int)required_bytes)) {
	// 		return read(buffer, buffer_size);
	// 	}
	// 
	// 	if (bytes_available < (int)required_bytes) {
	// 
	// 		if (timeout_us > 0) {
	// 			const uint64_t elapsed_us = hrt_elapsed_time(&start_time_us);
	// 
	// 			if (elapsed_us > timeout_us) {
	// 				//PX4_WARN("readAtLeast timeout %d bytes available (%llu us elapsed)", bytes_available, elapsed_us);
	// 				return -1;
	// 			}
	// 		}
	// 
	// 		int desired_bytes = required_bytes - bytes_available;
	// 
	// 		uint32_t sleeptime_us = desired_bytes * 1'000'000 / (_baudrate / 10);
	// 
	// 		if (desired_bytes == 1 || sleeptime_us <= 1000) {
	// 
	// 			int poll_timeout_ms = 0;
	// 
	// 			if (timeout_us > 0) {
	// 				poll_timeout_ms = timeout_us / 1000;
	// 			}
	// 
	// 			pollfd fds[1];
	// 			fds[0].fd = _serial_fd;
	// 			fds[0].events = POLLIN;
	// 
	// 			int poll_ret = ::poll(fds, 1, poll_timeout_ms);
	// 
	// 			if (poll_ret > 0) {
	// 				if (fds[0].revents & POLLIN) {
	// 					// There is data to read
	// 				}
	// 			}
	// 
	// 		} else {
	// 			if (timeout_us > 0) {
	// 				sleeptime_us = math::min(sleeptime_us, timeout_us);
	// 			}
	// 
	// 			//PX4_INFO("%s %d/%d bytes available, sleep time %" PRIu32 "us", _port, bytes_available, required_bytes, sleeptime_us);
	// 
	// 			px4_usleep(sleeptime_us);
	// 		}
	// 	}
	// }

	return -1;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
	if (!_open) {
		open();
	}

	if (!_configured) {
		configure();
	}

	int ret_write = qurt_uart_write(_serial_fd, (const char*) buffer, buffer_size);

	if (ret_write < 0) {
		PX4_ERR("%s write error %d", _port, ret_write);

	} else {
		_bytes_written += ret_write;
	}

	return ret_write;
}

const char *SerialImpl::getPort() const
{
	return _port;
}

bool SerialImpl::setPort(const char *port)
{
	if (port) {

		if (strcmp(port, _port) == 0) {
			return true;
		}

		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';

		_configured = false;

		return true;
	}

	return false;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	// process baud rate change


	// check if already configured
	if (baudrate == _baudrate) {
		return true;
	}

	if (baudrate != 0) {
		_baudrate = baudrate;
		_configured = false;
		return true;
	}

	return false;
}

ByteSize SerialImpl::getBytesize() const
{
	return _bytesize;
}

void SerialImpl::setBytesize(ByteSize bytesize)
{
	if (bytesize != _bytesize) {
		_bytesize = bytesize;
		_configured = false;
	}
}

Parity SerialImpl::getParity() const
{
	return _parity;
}

void SerialImpl::setParity(Parity parity)
{
	if (parity != _parity) {
		_parity = parity;
		_configured = false;
	}
}

StopBits SerialImpl::getStopbits() const
{
	return _stopbits;
}

void SerialImpl::setStopbits(StopBits stopbits)
{
	if (stopbits != _stopbits) {
		_stopbits = stopbits;
		_configured = false;
	}
}

FlowControl SerialImpl::getFlowcontrol() const
{
	return _flowcontrol;
}

void SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
	if (flowcontrol != _flowcontrol) {
		_flowcontrol = flowcontrol;
		_configured = false;
	}
}

bool SerialImpl::configure()
{
	_configured = false;

	if (!isOpen()) {
		return false;
	}

	// ByteSize: termios CSIZE

	// StopBits: termios CSTOPB

	// Parity: termios PARENB, PARODD

	// FlowControl: termios CRTSCTS

	// VMIN = 0, VTIME = 0: No blocking, return immediately with what is available

	// baudrate: input speed (cfsetispeed)

	// baudrate: output speed (cfsetospeed)

	// tcsetattr: set attributes

	PX4_INFO("%s configured successfully", _port);
	_configured = true;
	return true;
}

} // namespace device
