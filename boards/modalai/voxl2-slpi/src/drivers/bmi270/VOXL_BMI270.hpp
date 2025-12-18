/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file VOXL_BMI270.hpp
 *
 * Driver for the Bosch BMI270 connected via SPI.
 *
 */

#pragma once

#include "Bosch_BMI270_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Bosch_BMI270;
static constexpr uint8_t ACC_CONF_OSR4_800HZ = 0x8B;
static constexpr uint8_t GYR_CONF_OSR4_800HZ = 0x8B;
static constexpr uint8_t ACC_CONF_OSR4_1600HZ = 0x8C; // ODR 1.6kHz (bits [3:0] = 0x0C) - accel max
static constexpr uint8_t GYR_CONF_OSR4_1600HZ = 0x8C; // ODR 1.6kHz (bits [3:0] = 0x0C)
static constexpr uint8_t GYR_CONF_OSR4_3200HZ = 0x8D; // ODR 3.2kHz (bits [3:0] = 0x0D)
static constexpr uint8_t GYR_CONF_NORMBW_NOISEPERF_6400HZ = 0xEE; // ODR 6.4kHz + normal BW + noise perf
static constexpr uint8_t ACC_CONF_NORM_1600Hz = 0xAC;
static constexpr uint8_t GYR_CONF_NORM_1600Hz = 0xEC;


class VOXL_BMI270 : public device::SPI, public I2CSPIDriver<VOXL_BMI270>
{
public:
	VOXL_BMI270(const I2CSPIDriverConfig &config);
	~VOXL_BMI270() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor configuration: single ODR for both gyro and accel
	static constexpr float IMU_ODR{1600.f};                // 1.6 kHz
	static constexpr float FIFO_SAMPLE_DT_US{1e6f / IMU_ODR}; // 625 us

	// Rates (Hz)
	static constexpr float GYRO_RATE{IMU_ODR};
	static constexpr float ACCEL_RATE{IMU_ODR};

	// Common driver "rate" in Hz (what the rest of the driver/EKF think the IMU runs at)
	static constexpr uint32_t RATE{static_cast<uint32_t>(IMU_ODR)};

	// Sample period (same for gyro & accel here)
	// NOTE: if the rest of your driver expects microseconds, keep these in us.
	// If it expects seconds, divide by 1e6f when you use them.
	static constexpr float GYRO_SAMPLE_DT{FIFO_SAMPLE_DT_US};
	static constexpr float ACCEL_SAMPLE_DT{FIFO_SAMPLE_DT_US};
	static constexpr float FIFO_SAMPLE_DT{FIFO_SAMPLE_DT_US};


	// Scale factors for 16-bit data
	static constexpr float ACCEL_RANGE_G{16.f};
	static constexpr float ACCEL_RANGE_MS2{ACCEL_RANGE_G * CONSTANTS_ONE_G};
	static constexpr float ACCEL_SCALE_16BIT_16G{ACCEL_RANGE_MS2 / 32768.f};
	static constexpr float GYRO_RANGE_DPS{2000.f};
	static constexpr float GYRO_RANGE_RAD{math::radians(GYRO_RANGE_DPS)};
	static constexpr float GYRO_SCALE_16BIT_2000DPS{GYRO_RANGE_RAD / 32768.f};

	static constexpr uint8_t ID_088 = 0x1E;
	static constexpr uint8_t ID_090L = 0x1A;

	// Match ICM42688P's FIFO_MAX_SAMPLES for consistent behavior
	static constexpr int32_t FIFO_MAX_SAMPLES{10};

	hrt_abstime _temperature_update_timestamp{0};

	struct FIFOLengthReadBuffer
	{
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ};
		uint8_t dummy{0};
		uint8_t FIFO_LENGTH_0{0};
		uint8_t FIFO_LENGTH_1{0};
	};

	struct FIFOReadBuffer
	{
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ};
		uint8_t dummy{0};
		FIFO::Data f[FIFO_MAX_SAMPLES]{};
	};

	// ensure no struct padding
	static_assert(sizeof(FIFOReadBuffer) == (2 + FIFO_MAX_SAMPLES * sizeof(FIFO::Data)), "FIFOReadBuffer incorrect size");

	struct register_config_t
	{
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();

	void ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *gyro_frame);
	void ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *accel_frame);

	bool readAccelFrame(FIFO::Data *accel_frame);
	bool readGyroFrame(FIFO::Data *gyro_frame);

	void ConfigurePwr();
	void SetAccelScaleAndRange();
	void SetGyroScale();

	void CheckErrorRegister();

	void ConfigureFifo();
	void ConfigureSampleRate(int sample_rate = 0);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	bool LoadFeatureConfigAndVerify();

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadCount();
	// bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes);
	void FIFOReset();

	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME ": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME ": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME ": FIFO reset")};

	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t
	{
		WAIT_FOR_RESET,
		RESET,
		MICROCODE_LOAD,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250};														  // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / RATE))}; // checks out to be 2 ...

	// Debug timing tracking
	hrt_abstime _last_fifo_read_timestamp{0};
	uint32_t _fifo_read_count{0};
	uint32_t _total_accel_samples{0};
	uint32_t _total_gyro_samples{0};
	uint32_t _total_fifo_bytes{0};

	// Track min/max for detecting inconsistencies
	hrt_abstime _min_read_interval{UINT64_MAX};
	hrt_abstime _max_read_interval{0};
	uint16_t _min_fifo_bytes{UINT16_MAX};
	uint16_t _max_fifo_bytes{0};
	uint8_t _min_accel_samples{UINT8_MAX};
	uint8_t _max_accel_samples{0};
	uint8_t _min_gyro_samples{UINT8_MAX};
	uint8_t _max_gyro_samples{0};
																								  /*
																								  uint8_t _checked_register{0};
																								  static constexpr uint8_t size_register_cfg{11};
																							  
																								  register_config_t _register_cfg[size_register_cfg] {
																									  // Register                        | Set bits, Clear bits
																									  { Register::PWR_CONF,          0, ACC_PWR_CONF_BIT::acc_pwr_save },
																									  { Register::PWR_CTRL,          PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en,  0 },
																									  { Register::ACC_CONF,              ACC_CONF_BIT::acc_bwp_Normal | ACC_CONF_BIT::acc_odr_1600, Bit1 | Bit0 },
																									  { Register::GYR_CONF,              GYR_CONF_BIT::gyr_odr_1k6 | GYR_CONF_BIT::gyr_flt_mode_normal | GYR_CONF_BIT::gyr_noise_hp | GYR_CONF_BIT::gyr_flt_hp, Bit0 | Bit1 | Bit4},
																									  { Register::ACC_RANGE,             ACC_RANGE_BIT::acc_range_16g, 0 },
																									  { Register::FIFO_WTM_0,            0, 0 },
																									  { Register::FIFO_WTM_1,            0, 0 },
																									  { Register::FIFO_CONFIG_0,         FIFO_CONFIG_0_BIT::BIT1_ALWAYS | FIFO_CONFIG_0_BIT::FIFO_mode, 0 },
																									  { Register::FIFO_CONFIG_1,         FIFO_CONFIG_1_BIT::BIT4_ALWAYS | FIFO_CONFIG_1_BIT::Acc_en | FIFO_CONFIG_1_BIT::Gyr_en, 0 },
																									  { Register::INT1_IO_CTRL,          INT1_IO_CONF_BIT::int1_out, 0 },
																									  { Register::INT_MAP_DATA,    INT1_INT2_MAP_DATA_BIT::int1_fwm, 0},
																								  };
																								  */
	uint8_t _checked_register{0};

	// PWR_CONF: disable advanced power save (clear acc_pwr_save bits)
	// PWR_CTRL: enable accel + gyro + temp
	// ACC_CONF: OSR4 bandwidth, ODR 800 Hz (match apps proc)
	// ACC_RANGE: ±16 g
	// GYR_CONF: ODR 800 Hz, normal /
	// GYR_RANGE: ±2000 dps
	// FIFO_DOWNS: no downsampling
	// FIFO_CONFIG_0: FIFO mode, overwrite old samples, required BIT1 = 1
	// FIFO_CONFIG_1: FIFO accel + gyro enabled, required BIT4 = 1
	static constexpr uint8_t size_register_cfg{13};
	register_config_t _register_cfg[size_register_cfg]{
		// Register
		{Register::PWR_CONF, 0, ACC_PWR_CONF_BIT::acc_pwr_save},

		{Register::PWR_CTRL, PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en, 0},

		{Register::ACC_CONF, ACC_CONF_NORM_1600Hz, 0x7F}, // clear full ODR field before setting (1.6kHz - accel max)

		{Register::ACC_RANGE, ACC_RANGE_BIT::acc_range_16g, 0},

		{Register::GYR_CONF, GYR_CONF_NORM_1600Hz, 0x7F}, // 1.6kHz

		{Register::GYR_RANGE, GYR_RANGE_BIT::gyr_range_2000_dps, 0},

		{Register::FIFO_DOWNS, FIFO_DOWNS_BIT::fifo_no_downsampling, 0},

		{Register::FIFO_WTM_0, 0, 0xFF}, // watermark set dynamically by ConfigureFIFOWatermark()

		{Register::FIFO_WTM_1, 0, 0xFF}, // watermark set dynamically by ConfigureFIFOWatermark()

		{Register::FIFO_CONFIG_0, FIFO_CONFIG_0_BIT::BIT1_ALWAYS | FIFO_CONFIG_0_BIT::FIFO_mode, 0},

		{Register::FIFO_CONFIG_1, FIFO_CONFIG_1_BIT::BIT4_ALWAYS | FIFO_CONFIG_1_BIT::Acc_en | FIFO_CONFIG_1_BIT::Gyr_en, 0},

		{Register::INT1_IO_CTRL, 0x08, 0}, // Enable INT1 output

		{Register::INT_MAP_DATA, INT1_INT2_MAP_DATA_BIT::int1_fwm, 0}, // Map FIFO watermark to INT1
	};
};
