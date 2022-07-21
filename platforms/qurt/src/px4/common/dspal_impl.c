#include <px4_platform_common/log.h>

// Missing DSPAL functions

// From unistd.h:
int fsync(int fd) {
    PX4_INFO("DSPAL FSYNC called\n");
    return 0;
}

// From ioctl.h
int ioctl(int fd, int request, void *argp) {
    PX4_INFO("DSPAL IOCTL called. %d, %d, %p\n", fd, request, argp);
    return 0;
}

