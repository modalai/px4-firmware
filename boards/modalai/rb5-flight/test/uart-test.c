// C library headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define TX_BUFFER_LEN 200

int main() {
    printf("uart-test starting\n");

    int uart_fd = open("/dev/ttyUSB0", O_RDWR);

    if (uart_fd < 0) {
        fprintf(stderr, "Failed to open serial port\n");
        return -1;
    }

    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if (tcgetattr(uart_fd, &tty) != 0) {
        fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(uart_fd);
        return -1;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 8;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(uart_fd);
        return -1;
    }

    uint8_t tx_buf[TX_BUFFER_LEN];
    for (int i = 0; i < TX_BUFFER_LEN; i++) tx_buf[i] = i;

    while(1) {
        int nwrite = write(uart_fd, tx_buf, sizeof(tx_buf));
        if (nwrite != sizeof(tx_buf)) {
            printf("Only wrote %d out of %lu bytes\n", nwrite, sizeof(tx_buf));
        }
        usleep(1000);
    }

    close(uart_fd);

    printf("uart-test ending\n");

    return 0;
}
