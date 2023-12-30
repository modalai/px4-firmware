/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file px4_serial.h
 *
 * Includes functions for serial devices
 */

#pragma once

#if defined(__PX4_QURT) // Qurt based platforms

#ifdef __cplusplus
extern "C" {
#endif

int px4_serial_access(const char *pathname, int mode);
int px4_serial_open(const char *pathname, int flags);
int px4_serial_close(int fd);
ssize_t px4_serial_read(int fd, void *buf, size_t count);
ssize_t px4_serial_write(int fd, const void *buf, size_t count);

#ifdef __cplusplus
}
#endif

#else // Non Qurt platforms

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define px4_serial_access 	_GLOBAL access
#define px4_serial_open 	_GLOBAL open
#define px4_serial_close 	_GLOBAL close
#define px4_serial_read 	_GLOBAL read
#define px4_serial_write 	_GLOBAL write

#endif

