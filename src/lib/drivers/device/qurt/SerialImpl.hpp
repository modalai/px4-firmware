

#pragma once

namespace device
{

class SerialImpl
{
public:
	SerialImpl() {};
	virtual ~SerialImpl() {};

	int open(const char *port, uint32_t baudrate);
	bool isOpen() const;

	int close();

	ssize_t read(uint8_t *buffer, size_t buffer_size);
	ssize_t readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us);

	ssize_t write(const void *buffer, size_t buffer_size);

	uint32_t getBaudrate() const;
	int setBaudrate(uint32_t baudrate);

private:
	int _uartFd{-1};
	bool _isOpen{false};
};

} // namespace device