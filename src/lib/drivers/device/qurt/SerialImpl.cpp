
#include "SerialImpl.hpp"
#include "uart.h"

#define ASYNC_UART_READ_WAIT_US 500

namespace device
{

int SerialImpl::open(const char *port, uint32_t baudrate)
{
    _uartFd = qurt_uart_open(port, baudrate);

    if (_uartFd >= 0) return 0;
    else return -1;
}

bool SerialImpl::isOpen() const
{
    return _isOpen;
}

int SerialImpl::close()
{
    _isOpen = false;
    return 0;
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
    if (_uartFd >= 0) return qurt_uart_read(_uartFd, (char*) buffer, buffer_size, ASYNC_UART_READ_WAIT_US);
    else return 0;
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count = 1, uint32_t timeout_us = 0)
{
    return 0;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
    if (_uartFd >= 0) return qurt_uart_write(_uartFd, (const char *) buffer, buffer_size);
    else return 0;
}

uint32_t SerialImpl::getBaudrate() const
{
    return 0;
}

int SerialImpl::setBaudrate(uint32_t baudrate)
{
    return 0;
}

}