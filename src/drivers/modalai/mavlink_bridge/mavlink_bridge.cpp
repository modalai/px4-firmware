/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <string>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <v2.0/standard/mavlink.h>
#include "uORB/uORBManager.hpp"
#include <uORB/topics/mavlink_msg.h>

#include <px4_log.h>

extern "C" { __EXPORT int mavlink_bridge_main(int argc, char *argv[]); }

namespace mavlink_bridge
{

#define UDP_READ_BUF_LEN (32*1024)

static int sockfd_vvpx4;
static struct sockaddr_in vvpx4_addr;

bool debug = false;

int start(int argc, char *argv[]);
int stop();
int status();
void usage();

void recv_task(int argc, char *argv[]) {
	int i, bytes_read, msg_received;
	char buf[UDP_READ_BUF_LEN];
	struct sockaddr_in si_other;
	socklen_t slen = sizeof(si_other);
	mavlink_message_t msg;
	mavlink_status_t status;

	if (debug) PX4_INFO("Recv task starting");

	while (true) {
		// Receive UDP message from voxl-vision-px4, this is blocking with timeout
		bytes_read = recvfrom(sockfd_vvpx4, buf, UDP_READ_BUF_LEN, MSG_WAITALL,
							  (struct sockaddr*) &si_other, &slen);

		if (bytes_read < 0) {
 			if (errno != EAGAIN) PX4_ERR("ERROR: UDP recvfrom had a problem");
			// EAGAIN error means the timeout worked
			else if (debug) PX4_INFO("Recv task udp timeout");
		} else if (bytes_read > 0) {
			msg.msgid = msg.sysid = 0;
			for (i = 0; i < bytes_read; i++) {
				msg_received = mavlink_parse_char(0, buf[i], &msg, &status);

				// check for dropped packets
				// TODO CHECK THIS, maybe bigger read buffer?
				if (status.packet_rx_drop_count != 0) {
					if (debug) PX4_INFO("WARNING: UDP listener dropped %d packets", status.packet_rx_drop_count);
				}

				// msg_received indicates this byte was the end of a complete packet
				if (msg_received) {
					if (debug) {
						PX4_INFO("UDP recv msg ID: %d sysid:%d from port:%d IP: %s", \
								 msg.msgid, msg.sysid, ntohs(si_other.sin_port), \
								 inet_ntoa(si_other.sin_addr));

						if (i != bytes_read - 1) {
							PX4_INFO("Warning: more than one Mavlink packet in UDP packet. %d %d", i, bytes_read);
							PX4_INFO("         Suggest setting udp_mtu to 0 in voxl-vision-px4");
						}
					}
				}

				if (( ! msg_received) && (i == bytes_read - 1) && (debug)) {
					PX4_INFO("Detected unused bytes at end of UDP packet");
				}
			}
		} else if (debug) PX4_INFO("recvfrom returned 0");
	}

	if (debug) PX4_INFO("Recv task ending");
}

void task_main(int argc, char *argv[]) {
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			debug = true;
			PX4_INFO("Setting debug flag on");
			break;
		default:
			break;
		}
	}

	if (debug) PX4_INFO("Main task starting");

	int ret = px4_task_spawn_cmd("mavlink_bridge_recv",
								 SCHED_DEFAULT,
								 SCHED_PRIORITY_DEFAULT,
								 2000,
								 (px4_main_t) &recv_task,
								 (char *const *) argv);
	if (ret < 0) {
		PX4_ERR("Receive task start failed");
		return;
	}

    int mavlink_rx_msg_fd = orb_subscribe(ORB_ID(mavlink_rx_msg));
	struct mavlink_msg_s incoming_msg;
    px4_pollfd_struct_t fds[1] = { { .fd = mavlink_rx_msg_fd,  .events = POLLIN } };

	while (true) {
    	px4_poll(fds, 2, 1000);
    	if (fds[0].revents & POLLIN) {
    		orb_copy(ORB_ID(mavlink_rx_msg), mavlink_rx_msg_fd, &incoming_msg);
			if (debug) PX4_INFO("Got incoming mavlink msg of length %u", incoming_msg.msg_len);
			sendto(sockfd_vvpx4, incoming_msg.msg, incoming_msg.msg_len,
			       MSG_CONFIRM, (const struct sockaddr*) &vvpx4_addr, sizeof(vvpx4_addr));
		}
	}

	if (debug) PX4_INFO("Main task ending");
}

int start(int argc, char *argv[])
{
	sockfd_vvpx4 = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd_vvpx4 < 0) {
		PX4_ERR("ERROR: UDP socket creation failed");
		return -1;
	}

	// set receive timeout for the socket
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 500000;
	setsockopt(sockfd_vvpx4, SOL_SOCKET, SO_RCVTIMEO, (const char*) &tv, sizeof tv);

	// Address of the QGC port in voxl-vision-px4
	vvpx4_addr.sin_family = AF_INET;
	vvpx4_addr.sin_port = htons(14550);
	vvpx4_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	int ret = px4_task_spawn_cmd("mavlink_bridge_main",
								 SCHED_DEFAULT,
								 SCHED_PRIORITY_DEFAULT,
								 2000,
								 (px4_main_t) &task_main,
								 (char *const *) argv);

	if (ret < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

void usage() {
	PX4_INFO("Usage: mavlink_bridge start");
}

}

int mavlink_bridge_main(int argc, char *argv[]) {
	int myoptind = 1;

	if (argc <= 1) {
		mavlink_bridge::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return mavlink_bridge::start(argc - 1, argv + 1);
	} else {
		mavlink_bridge::usage();
		return -1;
	}
}
