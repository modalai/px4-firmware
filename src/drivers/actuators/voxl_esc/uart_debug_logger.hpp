/****************************************************************************
 *
 *   Copyright (c) 2024-2026 ModalAI, Inc. All rights reserved.
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
 * 3. Neither the name ModalAI nor the names of its contributors may be
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

#pragma once

#include <cstdint>
#include <cstdio>
#include <pthread.h>

/**
 * Non-blocking binary file logger with ring buffer and background flush thread.
 *
 * Designed for logging raw UART traffic without blocking the caller. Data is
 * written into a circular buffer by the caller (hot path) and flushed to disk
 * by a separate thread at a configurable interval.
 *
 * Usage:
 *   UartDebugLogger logger;
 *   logger.start("voxl_esc_log");       // opens unique file, starts thread
 *   logger.log_data(data, size);         // non-blocking, mutex-protected
 *   logger.stop();                       // joins thread, flushes, closes file
 *
 * Thread safety: log_data() is safe to call from any thread. start()/stop()
 * must be called from the owning thread.
 */
class UartDebugLogger
{
public:
	/**
	 * @param buffer_size       Ring buffer size in bytes (default 16KB).
	 * @param flush_interval_us Interval between flush attempts in microseconds
	 *                          (default 50ms).
	 */
	explicit UartDebugLogger(uint32_t buffer_size = 16 * 1024,
				 uint32_t flush_interval_us = 50000);
	~UartDebugLogger();

	UartDebugLogger(const UartDebugLogger &) = delete;
	UartDebugLogger &operator=(const UartDebugLogger &) = delete;

	/**
	 * Start the logger. Opens a uniquely-named file and starts the background
	 * flush thread.
	 *
	 * @param file_prefix Path prefix for log files. Files are named
	 *                    <prefix>_000.bin through <prefix>_099.bin.
	 * @return 0 on success, -1 on failure.
	 */
	int start(const char *file_prefix);

	/**
	 * Stop the logger. Signals the flush thread to exit, waits for it,
	 * flushes remaining data, and closes the file.
	 */
	void stop();

	/**
	 * Write data into the ring buffer. This is the hot-path function intended
	 * to be called from time-critical code. If the data exceeds the remaining
	 * buffer capacity, the oldest unread data will be silently overwritten.
	 *
	 * @param data Pointer to the data to log.
	 * @param size Number of bytes.
	 */
	void log_data(const uint8_t *data, uint32_t size);

	bool is_running() const { return _running; }

private:
	static void *flush_thread_entry(void *arg);
	void flush_thread_main();
	void flush_to_file();
	FILE *open_unique_file(const char *prefix);

	uint8_t        *_buffer{nullptr};
	const uint32_t  _buffer_size;
	const uint32_t  _flush_interval_us;

	volatile uint32_t _write_pos{0};
	volatile uint32_t _read_pos{0};

	pthread_mutex_t _mutex;

	FILE           *_file{nullptr};
	pthread_t       _thread{};
	volatile bool   _running{false};
	volatile bool   _exit_requested{false};
};
