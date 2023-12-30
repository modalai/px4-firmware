/****************************************************************************
 *
 * Copyright (C) 2023 ModalAI, Inc. All rights reserved.
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

#ifndef __ASM_GENERIC_POLL_H
#define __ASM_GENERIC_POLL_H

// #define POLLIN 0x0001
// #define POLLPRI 0x0002
// #define POLLOUT 0x0004
// #define POLLERR 0x0008
// #define POLLHUP 0x0010
// #define POLLNVAL 0x0020
// #define POLLRDNORM 0x0040
// #define POLLRDBAND 0x0080
// #ifndef POLLWRNORM
// #define POLLWRNORM 0x0100
// #endif
// #ifndef POLLWRBAND
// #define POLLWRBAND 0x0200
// #endif
// #ifndef POLLMSG
// #define POLLMSG 0x0400
// #endif
// #ifndef POLLREMOVE
// #define POLLREMOVE 0x1000
// #endif
// #ifndef POLLRDHUP
// #define POLLRDHUP 0x2000
// #endif
// #define POLLFREE (__force __poll_t) 0x4000
// #define POLL_BUSY_LOOP (__force __poll_t) 0x8000
struct pollfd {
  int fd;
  short events;
  short revents;
};

typedef unsigned int nfds_t;

#ifdef __cplusplus
extern "C" {
#endif

int poll(struct pollfd *fds, nfds_t nfds, int timeout);

#ifdef __cplusplus
}
#endif

#endif

