#include <poll.h>
#include <termios.h>
#include <px4_log.h>

int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	PX4_ERR("Undefined poll called");
	return -1;
}

int tcgetattr(int fd, struct termios *termios_p)
{
	PX4_ERR("Undefined tcgetattr called");
	return -1;
}

int cfsetispeed(struct termios *termios_p, speed_t speed)
{
	PX4_ERR("Undefined cfsetispeed called");
	return -1;
}

int cfsetospeed(struct termios *termios_p, speed_t speed)
{
	PX4_ERR("Undefined cfsetospeed called");
	return -1;
}

// int cfsetspeed(struct termios *termios_p, speed_t speed);

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	PX4_ERR("Undefined tcsetattr called");
	return -1;
}

// int tcflush(int fd, int queue_selector);

int px4_serial_access(const char *pathname, int mode)
{
	PX4_ERR("Undefined px4_serial_access called");
	return 0;
	// return -1;
}

int px4_serial_open(const char *pathname, int flags)
{
	PX4_ERR("Undefined px4_serial_open called");
	return 0;
	// return -1;
}

int px4_serial_close(int fd)
{
	PX4_ERR("Undefined px4_serial_close called");
	return 0;
	// return -1;
}

ssize_t px4_serial_read(int fd, void *buf, size_t count)
{
	PX4_ERR("Undefined px4_serial_read called");
	return 0;
	// return -1;
}

ssize_t px4_serial_write(int fd, const void *buf, size_t count)
{
	PX4_ERR("Undefined px4_serial_write called");
	return 0;
	// return -1;
}
