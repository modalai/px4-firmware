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

#include "uart_debug_logger.hpp"

#include <cstring>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

UartDebugLogger::UartDebugLogger(uint32_t buffer_size, uint32_t flush_interval_us)
	: _buffer_size(buffer_size)
	, _flush_interval_us(flush_interval_us)
{
	pthread_mutex_init(&_mutex, nullptr);
}

UartDebugLogger::~UartDebugLogger()
{
	if (_running) {
		stop();
	}

	pthread_mutex_destroy(&_mutex);
}

int UartDebugLogger::start(const char *file_prefix)
{
	if (_running) {
		return -1;
	}

	_buffer = new uint8_t[_buffer_size];

	if (!_buffer) {
		PX4_ERR("UartDebugLogger: failed to allocate %u byte buffer", _buffer_size);
		return -1;
	}

	_write_pos = 0;
	_read_pos = 0;
	_exit_requested = false;

	_file = open_unique_file(file_prefix);

	if (!_file) {
		delete[] _buffer;
		_buffer = nullptr;
		return -1;
	}

	_running = true;

	int ret = pthread_create(&_thread, nullptr, flush_thread_entry, this);

	if (ret != 0) {
		PX4_ERR("UartDebugLogger: failed to create flush thread (%d)", ret);
		fclose(_file);
		_file = nullptr;
		delete[] _buffer;
		_buffer = nullptr;
		_running = false;
		return -1;
	}

	pthread_setname_np(_thread, "esc_dbg_flush");

	return 0;
}

void UartDebugLogger::stop()
{
	if (!_running) {
		return;
	}

	_exit_requested = true;
	pthread_join(_thread, nullptr);

	// Final flush of any remaining data
	flush_to_file();

	if (_file) {
		fclose(_file);
		_file = nullptr;
	}

	delete[] _buffer;
	_buffer = nullptr;
	_running = false;
}

void UartDebugLogger::log_data(const uint8_t *data, uint32_t size)
{
	if (!_running || size == 0) {
		return;
	}

	if (size > _buffer_size) {
		size = _buffer_size;
	}

	pthread_mutex_lock(&_mutex);

	uint32_t space_to_end = _buffer_size - _write_pos;

	if (size > space_to_end) {
		memcpy(&_buffer[_write_pos], data, space_to_end);
		memcpy(&_buffer[0], data + space_to_end, size - space_to_end);

	} else {
		memcpy(&_buffer[_write_pos], data, size);
	}

	_write_pos = (_write_pos + size) % _buffer_size;

	pthread_mutex_unlock(&_mutex);
}

void *UartDebugLogger::flush_thread_entry(void *arg)
{
	static_cast<UartDebugLogger *>(arg)->flush_thread_main();
	return nullptr;
}

void UartDebugLogger::flush_thread_main()
{
	PX4_INFO("UartDebugLogger: flush thread started");

	while (!_exit_requested) {
		px4_usleep(_flush_interval_us);
		flush_to_file();
	}

	PX4_INFO("UartDebugLogger: flush thread exiting");
}

void UartDebugLogger::flush_to_file()
{
	pthread_mutex_lock(&_mutex);
	uint32_t write_pos = _write_pos;
	uint32_t read_pos = _read_pos;
	pthread_mutex_unlock(&_mutex);

	uint32_t available = (write_pos - read_pos + _buffer_size) % _buffer_size;

	if (available == 0 || !_file) {
		return;
	}

	uint32_t space_to_end = _buffer_size - read_pos;

	if (available > space_to_end) {
		fwrite(&_buffer[read_pos], 1, space_to_end, _file);
		fwrite(&_buffer[0], 1, available - space_to_end, _file);

	} else {
		fwrite(&_buffer[read_pos], 1, available, _file);
	}

	_read_pos = (read_pos + available) % _buffer_size;
}

FILE *UartDebugLogger::open_unique_file(const char *prefix)
{
	char path[128];
	FILE *f = nullptr;

	for (int i = 0; i < 100; i++) {
		snprintf(path, sizeof(path), "/data/px4/slpi/%s_%03d.bin", prefix, i);

		f = fopen(path, "r");

		if (f) {
			fclose(f);
			f = nullptr;
			continue;
		}

		f = fopen(path, "wb");

		if (f) {
			PX4_INFO("UartDebugLogger: opened %s", path);
			return f;
		}
	}

	PX4_ERR("UartDebugLogger: could not open log file");
	return nullptr;
}
