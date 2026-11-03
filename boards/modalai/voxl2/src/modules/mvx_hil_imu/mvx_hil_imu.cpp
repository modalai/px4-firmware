/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, inc. All rights reserved.
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
 * mvx_hil_imu: batched high-rate HIL IMU input for voxl-mvx-simulator.
 *
 * Only started by voxl-px4-sim-start (sim mode). Receives batches of IMU
 * samples from the simulator over localhost UDP and publishes them with
 * PX4Gyroscope/PX4Accelerometer::updateFIFO, the same path the real IMU
 * drivers use. sensor_gyro_fifo / sensor_accel_fifo then cross muorb a few
 * hundred times per second while carrying the full sample rate (e.g. 800 Hz
 * in batches of 4), with the simulator's per-sample timestamps.
 *
 * Packet (little endian, packed):
 *   header: char magic[4] = "MVXI"; uint8 version = 1; uint8 n (1..32);
 *           uint16 reserved; uint32 seq; float dt_us; float temperature_c
 *   n samples: uint64 t_us (CLOCK_MONOTONIC, oldest first);
 *              float gyro_rad_s[3]; float accel_m_s2[3]   (FRD body frame)
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace
{

struct __attribute__((packed)) MvxImuHeader {
	char magic[4];
	uint8_t version;
	uint8_t n;
	uint16_t reserved;
	uint32_t seq;
	float dt_us;
	float temperature_c;
};

struct __attribute__((packed)) MvxImuSample {
	uint64_t t_us;
	float gyro[3];
	float accel[3];
};

constexpr int kMaxSamples = 32;
constexpr int kDefaultPort = 14610;

// 1311244 (0x14020C): DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 2, TYPE: SIMULATION. Distinct from the
// mavlink HIL IMU (1310988, ADDR 1) so the sensors module never confuses the two sources.
constexpr uint32_t kDeviceId = 1311244;

// Ranges and int16 scaling, chosen like a real IMU (+-2000 deg/s, +-16 g)
constexpr float kGyroRange = 2000.f * M_PI_F / 180.f;
constexpr float kAccelRange = 16.f * CONSTANTS_ONE_G;
constexpr float kGyroScale = kGyroRange / 32768.f;
constexpr float kAccelScale = kAccelRange / 32768.f;

// Samples whose mapped time is further than this from now are re-stamped on receipt
constexpr int64_t kMaxClockErrorUs = 50000;

inline int16_t to_i16(float v, float scale)
{
	const float s = roundf(v / scale);
	return (int16_t)math::constrain(s, -32768.f, 32767.f);
}

uint64_t monotonic_us()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

} // namespace

class MvxHilImu : public ModuleBase
{
public:
	static Descriptor desc;

	explicit MvxHilImu(int port) : _port(port) {}
	~MvxHilImu() override
	{
		if (_fd >= 0) { close(_fd); }

		perf_free(_rx_perf);
		perf_free(_bad_perf);
		perf_free(_gap_perf);
		perf_free(_restamp_perf);
	}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);
	static MvxHilImu *instantiate(int argc, char *argv[]);

	void run() override;
	int print_status() override;

private:
	bool open_socket();
	void handle_packet(const uint8_t *buf, ssize_t len);

	int _port;
	int _fd{-1};

	PX4Accelerometer _px4_accel{kDeviceId};
	PX4Gyroscope _px4_gyro{kDeviceId};

	int64_t _clock_offset_us{0}; // hrt - CLOCK_MONOTONIC
	uint32_t _last_seq{0};
	bool _have_seq{false};
	uint64_t _samples{0};

	perf_counter_t _rx_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": packet interval")};
	perf_counter_t _bad_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packets")};
	perf_counter_t _gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": seq gaps")};
	perf_counter_t _restamp_perf{perf_alloc(PC_COUNT, MODULE_NAME": restamped")};
};

ModuleBase::Descriptor MvxHilImu::desc{task_spawn, custom_command, print_usage};

bool MvxHilImu::open_socket()
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		PX4_ERR("socket failed: %d", errno);
		return false;
	}

	int rcvbuf = 1 << 20;
	setsockopt(_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

	struct sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(_port);
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind 127.0.0.1:%d failed: %d", _port, errno);
		close(_fd);
		_fd = -1;
		return false;
	}

	return true;
}

void MvxHilImu::handle_packet(const uint8_t *buf, ssize_t len)
{
	if (len < (ssize_t)sizeof(MvxImuHeader)) { perf_count(_bad_perf); return; }

	MvxImuHeader h;
	memcpy(&h, buf, sizeof(h));

	if (memcmp(h.magic, "MVXI", 4) != 0 || h.version != 1 || h.n < 1 || h.n > kMaxSamples
	    || len != (ssize_t)(sizeof(MvxImuHeader) + h.n * sizeof(MvxImuSample))
	    || !(h.dt_us > 0.f)) {
		perf_count(_bad_perf);
		return;
	}

	perf_count(_rx_perf);

	if (_have_seq && h.seq != _last_seq + 1) {
		perf_count(_gap_perf);
	}

	_last_seq = h.seq;
	_have_seq = true;

	sensor_gyro_fifo_s gyro{};
	sensor_accel_fifo_s accel{};
	const MvxImuSample *s = (const MvxImuSample *)(buf + sizeof(MvxImuHeader));
	uint64_t t_last = 0;

	for (int i = 0; i < h.n; i++) {
		MvxImuSample smp;
		memcpy(&smp, &s[i], sizeof(smp));
		gyro.x[i] = to_i16(smp.gyro[0], kGyroScale);
		gyro.y[i] = to_i16(smp.gyro[1], kGyroScale);
		gyro.z[i] = to_i16(smp.gyro[2], kGyroScale);
		accel.x[i] = to_i16(smp.accel[0], kAccelScale);
		accel.y[i] = to_i16(smp.accel[1], kAccelScale);
		accel.z[i] = to_i16(smp.accel[2], kAccelScale);
		t_last = smp.t_us;
	}

	// timestamp_sample is the newest sample, as in the real IMU drivers
	const hrt_abstime now = hrt_absolute_time();
	int64_t t = (int64_t)t_last + _clock_offset_us;

	if (t <= 0 || llabs(t - (int64_t)now) > kMaxClockErrorUs) {
		perf_count(_restamp_perf);
		t = now;
	}

	gyro.timestamp_sample = (hrt_abstime)t;
	gyro.dt = h.dt_us;
	gyro.samples = h.n;
	accel.timestamp_sample = (hrt_abstime)t;
	accel.dt = h.dt_us;
	accel.samples = h.n;

	if (PX4_ISFINITE(h.temperature_c)) {
		_px4_gyro.set_temperature(h.temperature_c);
		_px4_accel.set_temperature(h.temperature_c);
	}

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);
	_samples += h.n;
}

void MvxHilImu::run()
{
	_px4_gyro.set_range(kGyroRange);
	_px4_gyro.set_scale(kGyroScale);
	_px4_accel.set_range(kAccelRange);
	_px4_accel.set_scale(kAccelScale);

	_clock_offset_us = (int64_t)hrt_absolute_time() - (int64_t)monotonic_us();
	PX4_INFO("listening on 127.0.0.1:%d, hrt - CLOCK_MONOTONIC = %lld us", _port, (long long)_clock_offset_us);

	uint8_t buf[sizeof(MvxImuHeader) + kMaxSamples * sizeof(MvxImuSample) + 16];

	while (!should_exit()) {
		struct pollfd fds {};
		fds.fd = _fd;
		fds.events = POLLIN;

		if (poll(&fds, 1, 100) <= 0) { continue; }

		// drain everything that is queued
		for (;;) {
			const ssize_t len = recv(_fd, buf, sizeof(buf), MSG_DONTWAIT);

			if (len < 0) { break; }

			handle_packet(buf, len);
		}
	}
}

int MvxHilImu::print_status()
{
	PX4_INFO("port %d, samples %llu, clock offset %lld us", _port, (unsigned long long)_samples,
		 (long long)_clock_offset_us);
	perf_print_counter(_rx_perf);
	perf_print_counter(_bad_perf);
	perf_print_counter(_gap_perf);
	perf_print_counter(_restamp_perf);
	return 0;
}

int MvxHilImu::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return MvxHilImu::instantiate(ac, av);
	}, argc, argv);
}

int MvxHilImu::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("mvx_hil_imu",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 5,
					  2400,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

MvxHilImu *MvxHilImu::instantiate(int argc, char *argv[])
{
	int port = kDefaultPort;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'p') { port = atoi(myoptarg); }
	}

	MvxHilImu *instance = new MvxHilImu(port);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (!instance->open_socket()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

int MvxHilImu::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MvxHilImu::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Batched high-rate HIL IMU input for voxl-mvx-simulator sim mode. Receives
batches of IMU samples over localhost UDP and publishes them as
sensor_gyro_fifo / sensor_accel_fifo (and the integrated sensor_gyro /
sensor_accel), exactly like a real FIFO IMU driver.
Started only by voxl-px4-sim-start.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mvx_hil_imu", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('p', 14610, 1, 65535, "UDP port on 127.0.0.1", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mvx_hil_imu_main(int argc, char *argv[])
{
	return ModuleBase::main(MvxHilImu::desc, argc, argv);
}
