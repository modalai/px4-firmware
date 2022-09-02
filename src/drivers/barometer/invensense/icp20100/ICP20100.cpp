/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ICP20100.hpp"

using namespace time_literals;

ICP20100::ICP20100(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

ICP20100::~ICP20100()
{
	perf_free(_reset_perf);
	perf_free(_sample_perf);
	perf_free(_bad_transfer_perf);
}

int
ICP20100::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool
ICP20100::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void
ICP20100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfer_perf);
}

int
ICP20100::init_boot_sequence_rev_a()
{
	uint8_t temp = 0;

	//
	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	switch (_rev_a_init_state) {
	case REV_A_INIT_STATE::NONE: {
			// Section 6.5 Step 4
			// Check the value from register regMap. OTP_STATUS2. BOOT_UP_STATUS
			//    - If 1, ICP-20100 didn’t go through power cycle after previous boot up sequence. No further
			//    initialization is required.
			if (read_register(Register::OTP_STATUS2, (uint8_t *)&temp)) {
				PX4_ERR("Failed to read OTP_STATUS2");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;

			}

			if (temp & OTP_STATUS2_BIT::BOOT_UP_STATUS) {
				PX4_INFO("Already configured");
				_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;
			}

			//    - If 0, boot up config is not done after ICP-20100 power on. Continue to step 5

			//
			// Section 6.5 Step 5
			// Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers
			// - regMap.MODE_SELECT.POWER_MODE = 1
			// - Wait 4ms;

			temp = 0;
			if (read_register(Register::MODE_SELECT, (uint8_t *)&temp)) {
				PX4_ERR("Failed to read MODE_SELECT");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}

			temp |= MODE_SELECT_BIT::POWER_MODE_ACTIVE;

			if (write_register(Register::MODE_SELECT, temp)) {
				PX4_ERR("Failed to write MODE_SELECT.POWER_MODE = 1");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			} else {
				_rev_a_init_state = REV_A_INIT_STATE::UNLOCK;
				ScheduleDelayed(4_ms);
			}

		}
		break;

	case REV_A_INIT_STATE::UNLOCK: {

			// Section 6.5 Step 6
			//  Unlock the main registers
			// - regMap.MASTER_LOCK.LOCK = 0x1f
			if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK)) {
				PX4_ERR("Failed to write MASTER_LOCK = 1");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			}

			// Section 6.5 Step 7
			//  Enable the OTP and the write switch
			// - regMap.OTP_CONFIG1.OTP_ENABLE = 1
			// - regMap.OTP_CONFIG1.OTP_WRITE_SWITCH = 1
			// - wait 10μs
			temp = OTP_CONFIG1_BIT::OTP_WR | OTP_CONFIG1_BIT::OTP_EN;
			if (write_register(Register::OTP_CONFIG1, temp)) {
				PX4_ERR("Failed to write OTP_CONFIG1");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			}
			usleep(10);

			// Section 6.5 Step 8
			//  Toggle the OTP reset pin
			// - regMap.OTP_DBG2.RESET = 1
			// - wait 10us
			// - regMap.OTP_DBG2.RESET = 0
			// - wait 10us

			temp = OTP_DBG2_BIT::RESET;
			if (write_register(Register::OTP_DBG2, temp)) {
				PX4_ERR("Failed to write OTP_DBG2");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			}
			usleep(10);
			temp = 0;
			if (write_register(Register::OTP_DBG2, temp)) {
				PX4_ERR("Failed to write OTP_DBG2");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			usleep(10);

			// Section 6.5 Step 9
			// Program redundant read
			// - regMap.OTP_MRA_LSB = 0x04
			// - regMap.OTP_MRA_MSB = 0x04
			// - regMap.OTP_MRB_LSB = 0x21
			// - regMap.OTP_MRB_MSB = 0x20
			// - regMap.OTP_MR_LSB = 0x10
			// - regMap.OTP_MR_MSB = 0x80
			if (write_register(Register::OTP_MRA_LSB, 0x04)) {
				PX4_ERR("Failed to write OTP_MRA_LSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (write_register(Register::OTP_MRA_MSB, 0x04)) {
				PX4_ERR("Failed to write OTP_MRA_MSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (write_register(Register::OTP_MRB_LSB, 0x21)) {
				PX4_ERR("Failed to write OTP_MRB_LSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (write_register(Register::OTP_MRB_MSB, 0x20)) {
				PX4_ERR("Failed to write OTP_MRB_MSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (write_register(Register::OTP_MR_LSB, 0x10)) {
				PX4_ERR("Failed to write OTP_MR_LSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (write_register(Register::OTP_MR_MSB, 0x80)) {
				PX4_ERR("Failed to write OTP_MR_MSB");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}

			_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;
		}

	default:
		break;

	}





	return PX4_OK;
}

int
ICP20100::probe()
{
	uint8_t temp = 0;

	//
	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	// Section 6.5 Step 2
	// Initialize the I2C interface by toggling the clock line a few times. The easiest way to do that is by
	// inserting a dummy I2C write transaction. You can, for example, execute the first transaction (write to
	// lock register) twice
	//
	if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK)) {
		return PX4_ERROR;
	}

	if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK)) {
		return PX4_ERROR;
	}

	if (read_register(Register::WHO_AM_I, (uint8_t *)&temp)) {
		PX4_ERR("Failed to read HW version");
		return PX4_ERROR;

	} else {
		if (temp != Product_ID) {
			PX4_ERR("Unexpected ID:  0x%02x", temp);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void
ICP20100::RunImpl()
{
	//const hrt_abstime now = hrt_absolute_time();

	switch (_state2) {
	case STATE2::INIT:
		/*
		 * From: DS-000416-ICP-20100-v1.4.pdf
		 *
		 * Rev A (Version = 0x00) needs extra initialization sequence
		 */
		uint8_t temp;

		if (read_register(Register::VERSION, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read HW version");
			_state2 = STATE2::ERROR;

		} else {
			if (temp == ICP_20100_HW_REVA_VERSION) {
				_state2 = STATE2::INIT_REV_A;

			} else if (temp == ICP_20100_HW_REVB_VERSION) {
				PX4_INFO("HW Rev B");
				_state2 = STATE2::READY;

			} else {
				PX4_ERR("Unkown HW revision");
				_state2 = STATE2::ERROR;
			}
		}

		break;

	case STATE2::INIT_REV_A: {
			if (init_boot_sequence_rev_a()) {
				PX4_ERR("Failed to initialize Rev A HW");
				_state2 = STATE2::ERROR;

			} else {
				if(_rev_a_init_state == REV_A_INIT_STATE::ERROR){
					PX4_ERR("Failed to initialize Rev A HW");
					_state2 = STATE2::ERROR;
				}
				if(_rev_a_init_state == REV_A_INIT_STATE::SUCCESS){
					_state2 = STATE2::READY;
				}
			}
		}
		break;

	case STATE2::READY: {
			__asm("nop");
		}
		break;

	case STATE2::MEASURE: {
			__asm("nop");
		}
		break;

	case STATE2::ERROR: {
			PX4_ERR("Fatal Error");
			exit(1);
		}
		break;
	}

	ScheduleDelayed(100_ms);

	/*
		switch (_state) {
		case STATE::RESET: {
				// Software Reset
				send_command(Cmd::SOFT_RESET);
				_reset_timestamp = now;
				_failure_count = 0;
				_state = STATE::WAIT_FOR_RESET;
				perf_count(_reset_perf);
				ScheduleDelayed(100_ms); // Power On Reset: max 100ms
			}
			break;

		case STATE::WAIT_FOR_RESET: {
				// check product id
				uint16_t ID = 0;
				read_response(Cmd::READ_ID, (uint8_t *)&ID, 2);
				uint8_t PROD_ID = (ID >> 8) & 0x3f; // Product ID Bits 5:0

				if (PROD_ID == Product_ID) {
					// if reset succeeded then read otp
					_state = STATE::READ_OTP;
					ScheduleDelayed(10_ms); // Time to coefficients are available.

				} else {
					// RESET not complete
					if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
						PX4_DEBUG("Reset failed, retrying");
						_state = STATE::RESET;
						ScheduleDelayed(100_ms);

					} else {
						PX4_DEBUG("Reset not complete, check again in 10 ms");
						ScheduleDelayed(10_ms);
					}
				}
			}
			break;

		case STATE::READ_OTP: {
				// read otp
				uint8_t addr_otp_cmd[3] = {0x00, 0x66, 0x9c};
				uint8_t otp_buf[3];
				uint8_t crc;
				bool success = true;

				send_command(Cmd::SET_ADDR, addr_otp_cmd, 3);

				for (uint8_t i = 0; i < 4; i++) {
					read_response(Cmd::READ_OTP, otp_buf, 3);

					crc = 0xFF;

					for (int j = 0; j < 2; j++) {
						crc = (uint8_t)cal_crc(crc, otp_buf[j]);
					}

					if (crc != otp_buf[2]) {
						success = false;
						break;
					}

					_scal[i] = (otp_buf[0] << 8) | otp_buf[1];
				}

				if (success) {
					_state = STATE::MEASURE;

				} else {
					_state = STATE::RESET;
				}

				ScheduleDelayed(10_ms);
			}
			break;

		case STATE::MEASURE:
			if (Measure()) {
				// if configure succeeded then start measurement cycle
				_state = STATE::READ;
				perf_begin(_sample_perf);
				ScheduleDelayed(_measure_interval);

			} else {
				// MEASURE not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Measure failed, resetting");
					_state = STATE::RESET;

				} else {
					PX4_DEBUG("Measure failed, retrying");
				}

				ScheduleDelayed(_measure_interval);
			}

			break;

		case STATE::READ: {
				uint8_t comp_data[9] {};
				bool success = false;

				if (read_measure_results(comp_data, 9) == PX4_OK) {
					perf_end(_sample_perf);

					uint16_t _raw_t = (comp_data[0] << 8) | comp_data[1];
					uint32_t L_res_buf3 = comp_data[3];	// expand result bytes to 32bit to fix issues on 8-bit MCUs
					uint32_t L_res_buf4 = comp_data[4];
					uint32_t L_res_buf6 = comp_data[6];
					uint32_t _raw_p = (L_res_buf3 << 16) | (L_res_buf4 << 8) | L_res_buf6;

					// constants for presure calculation
					static constexpr float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
					static constexpr float _lut_lower = 3.5 * 0x100000;	// 1<<20
					static constexpr float _lut_upper = 11.5 * 0x100000;	// 1<<20
					static constexpr float _quadr_factor = 1 / 16777216.0;
					static constexpr float _offst_factor = 2048.0;

					// calculate temperature
					float _temperature_C = -45.f + 175.f / 65536.f * _raw_t;

					// calculate pressure
					float t = (float)(_raw_t - 32768);
					float s1 = _lut_lower + (float)(_scal[0] * t * t) * _quadr_factor;
					float s2 = _offst_factor * _scal[3] + (float)(_scal[1] * t * t) * _quadr_factor;
					float s3 = _lut_upper + (float)(_scal[2] * t * t) * _quadr_factor;
					float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
						   s2 * s3 * (_pcal[1] - _pcal[2]) +
						   s3 * s1 * (_pcal[2] - _pcal[0])) /
						  (s3 * (_pcal[0] - _pcal[1]) +
						   s1 * (_pcal[1] - _pcal[2]) +
						   s2 * (_pcal[2] - _pcal[0]));
					float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
					float b = (_pcal[0] - a) * (s1 + c);
					float _pressure_Pa = a + b / (c + _raw_p);

					float temperature = _temperature_C;
					float pressure = _pressure_Pa;

					// publish
					sensor_baro_s sensor_baro{};
					sensor_baro.timestamp_sample = now;
					sensor_baro.device_id = get_device_id();
					sensor_baro.pressure = pressure;
					sensor_baro.temperature = temperature;
					sensor_baro.error_count = perf_event_count(_bad_transfer_perf);
					sensor_baro.timestamp = hrt_absolute_time();
					_sensor_baro_pub.publish(sensor_baro);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}

					_state = STATE::MEASURE;

				} else {
					perf_count(_bad_transfer_perf);
				}

				if (!success) {
					_failure_count++;

					// full reset if things are failing consistently
					if (_failure_count > 10) {
						Reset();
						return;
					}
				}

				ScheduleDelayed(1000_ms / 8 - _measure_interval); // 8Hz
			}

			break;
		}
		*/
}

bool
ICP20100::Measure()
{
	/*
	  From ds-000186-icp-101xx-v1.0.pdf, page 6, table 1

	  Sensor                  Measurement       Max Time
	  Mode                    Time (Forced)
	  Low Power (LP)             1.6 ms          1.8 ms
	  Normal (N)                 5.6 ms          6.3 ms
	  Low Noise (LN)             20.8 ms         23.8 ms
	  Ultra Low Noise(ULN)       83.2 ms         94.5 ms
	*/
	Cmd cmd;

	switch (_mode) {
	case MODE::FAST:
		cmd = Cmd::MEAS_LP;
		_measure_interval = 2_ms;
		break;

	case MODE::ACCURATE:
		cmd = Cmd::MEAS_LN;
		_measure_interval = 24_ms;
		break;

	case MODE::VERY_ACCURATE:
		cmd = Cmd::MEAS_ULN;
		_measure_interval = 95_ms;
		break;

	case MODE::NORMAL:
	default:
		cmd = Cmd::MEAS_N;
		_measure_interval = 7_ms;
		break;
	}

	if (send_command(cmd) != PX4_OK) {
		return false;
	}

	return true;
}

int8_t
ICP20100::cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x31;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

int
ICP20100::read_measure_results(uint8_t *buf, uint8_t len)
{
	return transfer(nullptr, 0, buf, len);
}

int
ICP20100::read_response(Cmd cmd, uint8_t *buf, uint8_t len)
{
	uint8_t buff[2];
	buff[0] = ((uint16_t)cmd >> 8) & 0xff;
	buff[1] = (uint16_t)cmd & 0xff;
	return transfer(&buff[0], 2, buf, len);
}
int
ICP20100::read_register(Register reg, uint8_t *buf)
{
	uint8_t buff[2];
	buff[0] = (uint8_t)reg;
	buff[1] = 0x00;
	return transfer(&buff[0], 2, buf, 1);
}

int
ICP20100::send_command(Cmd cmd)
{
	uint8_t buf[2];
	buf[0] = ((uint16_t)cmd >> 8) & 0xff;
	buf[1] = (uint16_t)cmd & 0xff;
	return transfer(buf, sizeof(buf), nullptr, 0);
}

int
ICP20100::send_command(Cmd cmd, uint8_t *data, uint8_t len)
{
	uint8_t buf[5];
	buf[0] = ((uint16_t)cmd >> 8) & 0xff;
	buf[1] = (uint16_t)cmd & 0xff;
	memcpy(&buf[2], data, len);
	return transfer(&buf[0], len + 2, nullptr, 0);
}
int
ICP20100::write_register(Register reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)reg;
	buf[1] = data;
	return transfer(&buf[0], 2, nullptr, 0);
}
