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
#include "VOXL_BMI270.hpp"
#include "bmi270_config.hpp"
#define VOXL_BMI270_DEBUG
#define VOXL_BMI270_MAX_FIFO_SAMPLES 8
using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

/**
* The following device config microcode has the following copyright:
*
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
// end of Bosch microcode copyright


VOXL_BMI270::VOXL_BMI270(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(0),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

VOXL_BMI270::~VOXL_BMI270()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int VOXL_BMI270::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	PX4_DEBUG("init function called, resetting...");


	return Reset() ? 0 : -1;
}

bool VOXL_BMI270::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

// Debug function that Ardupilot is equipped with
void VOXL_BMI270::CheckErrorRegister()
{
#ifdef VOXL_BMI270_DEBUG
	uint8_t err = RegisterRead(Register::ERR_REG);

	if (err) {
		if ((err & 1) == 1) {
			uint8_t status =  RegisterRead(Register::INTERNAL_STATUS);

			switch (status & 0xF) {
			case 0:
				PX4_DEBUG("VOXL_BMI270: not_init");
				break;

			case 2:
				PX4_DEBUG("VOXL_BMI270: init_err");
				break;

			case 3:
				PX4_DEBUG("VOXL_BMI270: drv_err");
				break;

			case 4:
				PX4_DEBUG("VOXL_BMI270: sns_stop");
				break;

			case 5:
				PX4_DEBUG("VOXL_BMI270: nvm_error");
				break;

			case 6:
				PX4_DEBUG("VOXL_BMI270: start_up_error");
				break;

			case 7:
				PX4_DEBUG("VOXL_BMI270: compat_error");
				break;

			case 1: // init ok
				if ((status >> 5 & 1) == 1) {
					PX4_DEBUG("VOXL_BMI270: axes_remap_error");

				} else if ((status >> 6 & 1) == 1) {
					PX4_DEBUG("VOXL_BMI270: odr_50hz_error");
				}

				break;
			}

		} else if ((err >> 6 & 1) == 1) {
			PX4_DEBUG("VOXL_BMI270: fifo_err");

		} else if ((err >> 7 & 1) == 1) {
			PX4_DEBUG("VOXL_BMI270: aux_err");

		} else {
			PX4_DEBUG("VOXL_BMI270: internal error detected %d", err >> 1 & 0xF);
		}
	}

#endif

}

void VOXL_BMI270::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void VOXL_BMI270::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int VOXL_BMI270::probe()
{
	// When starting communication with the VOXL_BMI270, according page 123 of the
	// datasheet a rising edge on CSB is required to start SPI.
	// It is recommended to just read the CHIP_ID register once to do that
	// but ignore the result.
	RegisterRead(Register::CHIP_ID);

	// It takes 200us for the device to start SPI communication.
	px4_usleep(200);

	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	if (CHIP_ID != chip_id) {
		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", CHIP_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void VOXL_BMI270::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		PX4_DEBUG("STATE: RESET");
		// 0xB6 is written to the CMD register for a soft reset
		RegisterWrite(Register::CMD, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // Following a delay of 1 ms, all configuration settings are overwritten with their reset value.
		break;



	case STATE::WAIT_FOR_RESET:
		PX4_DEBUG("STATE: WAIT FOR RESET");
		
		//Hardware initialization steps according to datasheet:
		//1. Disable PWR_CONF.adv_power_save and wait for 450us
		//2. Write 0x00 to INIT_CTRL
		//3. Burst write initialization file to INIT_DATA
		//4. Write 0x01 to INIT_CTRL
		//5. Wait 150ms and read register INTERNAL_STATUS for value 0b001
		//6. If step 5 passed, enter configure state


		
		if ((RegisterRead(Register::CHIP_ID) == chip_id)) {
			PX4_DEBUG("Read from CHIP_ID register and the IDs match, trying again");
			if ((RegisterRead(Register::CHIP_ID) == chip_id)) {
				PX4_DEBUG("chip Id read correctly again");
			}
			else {
				PX4_DEBUG("SECOND CHIP ID READ FAILED"); 
			}
			// 1. Disable PWR_CONF.adv_power_save and wait for 450u
			PX4_DEBUG("TEST: NOT skipping PWR_CONF write before MICROCODE_LOAD");

			uint8_t pwr = RegisterRead(Register::PWR_CONF);
			PX4_DEBUG("PWR_CONF before: 0x%02hhX", pwr);

			//PX4_DEBUG("TEST FOR BREAKING : write IF_CONF=0x00 (4-wire SPI)");
			//RegisterWrite(Register::PWR_CONF, 0x00);
			//RegisterWrite(Register::IF_CONF, 0x00);
			_state = STATE::MICROCODE_LOAD;
			ScheduleDelayed(450_us);

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

		break;

		
	case STATE::MICROCODE_LOAD:
		PX4_DEBUG("STATE: MICROCODE_LOAD (test stub, no config load)");
		if (RegisterRead(Register::CHIP_ID) == chip_id){
			PX4_DEBUG("CHIP ID READ IN MICROLOAD");
		}
		else {
			PX4_DEBUG("CHIP ID NOT ABLE TO READ IN MICROLOAD"); 
		}
		/*
		for (int i = 0; i < 10; ++i) {
			px4_usleep(1000);
			uint8_t cid = RegisterRead(Register::CHIP_ID);
			uint8_t err = RegisterRead(Register::ERR_REG);
			PX4_DEBUG("MICROCODE_LOAD test: CHIP_ID=0x%02hhX ERR_REG=0x%02hhX (i=%d)",
					cid, err, i);
		}

		_state = STATE::RESET;
		ScheduleDelayed(10_ms);
		break;

		
		{

			// 2. Write 0x00 to INIT_CTRL

			RegisterWrite(Register::INIT_CTRL, 0x00);
			// give it the maximum FIFO config file
			PX4_DEBUG("attempting to upload initialization file onto VOXL_BMI270");

			//  3. Burst write initialization file to INIT_DATA

			int res = transfer(bmi270_config_file, nullptr, bmi270_config_file_len);

			if (res == PX4_OK) {
				RegisterWrite(Register::INIT_CTRL, 1);
				PX4_DEBUG("Successfully uploaded initialization file onto VOXL_BMI270");
				PX4_DEBUG("Preparing to read INTERNAL_STATUS register");
				_state = STATE::CONFIGURE;
				ScheduleDelayed(150_ms);


			} else {
				PX4_DEBUG("Failed to upload initialization file onto VOXL_BMI270, resetting");
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}

			break;
		}
		*/

		if (LoadFeatureConfigAndVerify()) {
			PX4_DEBUG("feature config uploaded & verified; proceeding to CONFIGURE");
			_state = STATE::CONFIGURE;
			// You already waited while polling; no additional delay is needed.
			ScheduleNow();
		} else {
			PX4_DEBUG("feature config upload/verify failed, resetting");
			_state = STATE::RESET;
			ScheduleDelayed(10_ms);
		}
		break;
	
		
	case STATE::CONFIGURE:
		PX4_DEBUG("STATE: CONFIGURE");
		if (Configure()) {

			// if configure succeeded then start reading from FIFO

			if (DataReadyInterruptConfigure()) {
				PX4_INFO("bmi270 using interrupt\n"); 
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				PX4_INFO("bmi270 using no interrupt\n"); 
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();
			_state = STATE::FIFO_READ;


		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			PX4_DEBUG("STATE: FIFO");
			PX4_DEBUG("reading from FIFO");

			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				PX4_DEBUG("data ready interrupt enabled");
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}



			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			// more bytes than what the buffer takes so an overflow
			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);

			} else {

				uint8_t samples = fifo_count / sizeof(FIFO::Data);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= _fifo_gyro_samples) {
					if (FIFORead(timestamp_sample, fifo_count)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}

			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					PX4_DEBUG("failure count > 10, resetting...");
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					PX4_DEBUG("register check failed, resetting...");

					// register check failed, force reset
					perf_count(_bad_register_perf);
					// Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}

		}

		break;
	}
}

void VOXL_BMI270::SetAccelScaleAndRange()
{
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_2g:
		_px4_accel.set_scale(2.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case acc_range_4g:
		_px4_accel.set_scale(4.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case acc_range_8g:
		_px4_accel.set_scale(8.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case acc_range_16g:
		_px4_accel.set_scale(16.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

/*
bool VOXL_BMI270::LoadFeatureConfigAndVerify()
{
    // 1) Soft reset (already done in RESET state, but harmless here if you want it here too)
    RegisterWrite(Register::CMD, 0xB6); 
	px4_usleep(3000);

    // 2) Ensure 4-wire SPI if needed (optional; keep if your HW requires it)
    RegisterWrite(Register::IF_CONF, 0x00); // BMI270_4_WIRE_SPI_CMD
    px4_usleep(2000);

    // 3) Disable Advanced Power Save
    RegisterWrite(Register::PWR_CONF, 0x00);
    px4_usleep(2000);

    // 4) Put INIT engine into config-write mode
    RegisterWrite(Register::INIT_CTRL, 0x00);

    // 5) Upload in chunks (exactly like your apps driver)
    static constexpr uint16_t CHUNK = 2; // even, safe
    int res = PX4_OK;

    for (uint32_t i = 0; i < bmi270_config_file_len; i += CHUNK) {
        uint16_t wlen = (uint16_t)((bmi270_config_file_len - i < CHUNK) ? (bmi270_config_file_len - i) : CHUNK);
        if (wlen & 1) { wlen -= 1; } // keep even length

        if (wlen == 0){
			PX4_DEBUG("wlen == 0 breaking"); 
			break;
		} 

        // Address packing to MATCH your working apps-side code:
        // low nibble first, then the upper bits (word address = i/2)
		uint8_t addr_bytes[2];
        addr_bytes[0] = (uint8_t)((i / 2) & 0x0F);     // low 4 bits
        addr_bytes[1] = (uint8_t)((i / 2) >> 4);       // upper bits

        // Write INIT_ADDR_0 + INIT_ADDR_1 in one burst starting at INIT_ADDR_0
        uint8_t tx_addr[3];
		tx_addr[0] = (((uint8_t)Register::INIT_ADDR_0) & (0x7F)); // clear read/write bit
		tx_addr[1] = addr_bytes[0];
		tx_addr[2] = addr_bytes[1];
		uint8_t dummy_rx[sizeof(tx_addr)];
		res = transfer(tx_addr, dummy_rx, sizeof(tx_addr));
		if (res != PX4_OK) {
			PX4_DEBUG("INIT_ADDR write failed @i=%u", (unsigned)i);
			break;
		}
		px4_usleep(100); 
		uint8_t tx_data[1 + CHUNK];
		tx_data[0] = (((uint8_t)Register::INIT_DATA) & (0x7F));   // clear read/write bit
		memcpy(&tx_data[1], &bmi270_config_file[i], wlen);
		uint8_t dummy_rx2[1 + CHUNK];
		res = transfer(tx_data, dummy_rx2, (size_t)(1 + wlen));
		if (res != PX4_OK) {
			PX4_DEBUG("INIT_DATA write failed @i=%u", (unsigned)i);
			break;
		}
	
    }

    if (res != PX4_OK) {
		PX4_DEBUG("res != ok, exiting");
        return false;
    }
	

    // 6) Trigger load
    RegisterWrite(Register::INIT_CTRL, 0x01);

    // 7) Poll INTERNAL_STATUS until 0x01 or timeout (your exact behavior)
    uint8_t istat = 0;
    const int max_tries = 200; // 1 ms each ~ 200 ms
    int tries = 0;

    do {
        px4_usleep(1000);
        istat = RegisterRead(Register::INTERNAL_STATUS);
        tries++;
    } while (istat != 0x01 && tries < max_tries);

    PX4_DEBUG("INTERNAL_STATUS after load: 0x%02hhX (tries=%d)", istat, tries);

    if (istat != 0x01) {
        // Mirror your extra debug on failure
        uint8_t ierr = 0;     // INTERNAL_ERR (Bosch naming)
        uint8_t emsk = 0;     // ERR_REG_MSK (Bosch naming)
        uint8_t ictrl = 0;    // INIT_CTRL/INIT_CTRL

        // These three enum names must map to your actual register addresses:
        // INTERNAL_ERR (0x2A), ERR_REG_MSK (0x02?), INIT_CTRL (0x59)
        // If you don’t have INTERNAL_ERR / ERR_REG_MSK in your enum, add them.
        // Fallback: just don’t print them if you prefer minimal changes.
       
            // only if you’ve defined them in your Register enum:
    	ierr  = RegisterRead(Register::INTERNAL_ERR);
    	emsk  = RegisterRead(Register::ERR_REG_MSK);
    
        ictrl = RegisterRead(Register::INIT_CTRL);

        PX4_DEBUG("Config load FAILED. INTERNAL_ERR=0x%02hhX ERR_REG_MSK=0x%02hhX INIT_CTRL=0x%02hhX",
                  ierr, emsk, ictrl);
        return false;
    }

    return true;
}*/

bool VOXL_BMI270::LoadFeatureConfigAndVerify()
{
	PX4_DEBUG("=== LoadFeatureConfigAndVerify() BEGIN ===");
	px4_usleep(1000);
	// 0) Snapshot some state before we touch anything
	const uint8_t chip_id_read   = RegisterRead(Register::CHIP_ID);
	const uint8_t istat_before   = RegisterRead(Register::INTERNAL_STATUS);
	const uint8_t err_before     = RegisterRead(Register::ERR_REG);
	PX4_DEBUG("Pre-load snapshot: CHIP_ID=0x%02hhX INTERNAL_STATUS=0x%02hhX ERR_REG=0x%02hhX",
	          chip_id_read, istat_before, err_before);

	const uint8_t if_config = RegisterRead(Register::IF_CONF); 
	PX4_DEBUG("IF_CONF at start of load config: 0x%02hhX", if_config); 
	if (chip_id_read != chip_id) {   // 'chip_id' is the Bosch_BMI270::chip_id constant
        PX4_DEBUG("CHIP_ID mismatch at entry (got 0x%02hhX, expect 0x%02hhX) - polling...",
                  chip_id_read, chip_id);

        bool chip_ok = false;

        for (int tries = 0; tries < 20; ++tries) { // 20 x 1 ms = ~20 ms
            px4_usleep(1000);

            uint8_t chip_poll = RegisterRead(Register::CHIP_ID);
            uint8_t err_poll  = RegisterRead(Register::ERR_REG);

            // Log first few and then a couple of spaced-out samples so we don't spam too hard
            if (tries < 3 || tries == 9 || tries == 19 || chip_poll == chip_id) {
                PX4_DEBUG("CHIP_ID poll[%d]: CHIP_ID=0x%02hhX ERR_REG=0x%02hhX",
                          tries, chip_poll, err_poll);
            }

            if (chip_poll == chip_id) {
                PX4_DEBUG("CHIP_ID became valid (0x%02hhX) after %d ms",
                          chip_poll, (tries + 1));
                chip_ok = true;
                break;
            }
        }

        if (!chip_ok) {
            PX4_DEBUG("CHIP_ID never became valid during poll; continuing anyway");
        }
    }


	const uint8_t istat_after_reset = RegisterRead(Register::INTERNAL_STATUS);
	PX4_DEBUG("After soft reset: INTERNAL_STATUS=0x%02hhX", istat_after_reset);

	// 2) Ensure 4-wire SPI if needed (optional; keep if your HW requires it)
	PX4_DEBUG("Step 2: write IF_CONF=0x00 (4-wire SPI)");
	RegisterWrite(Register::IF_CONF, 0x00);
	px4_usleep(2000);
	const uint8_t if_conf_read = RegisterRead(Register::IF_CONF);
	PX4_DEBUG("IF_CONF readback: 0x%02hhX", if_conf_read);

	// 3) Disable Advanced Power Save
	PX4_DEBUG("Step 3: write PWR_CONF=0x02 (DISABLE APS AND leaves FSW as 1) and SLEEP FOR A LONG TIME BEFORE TRYING ANYTHING ELSE");
	RegisterWrite(Register::PWR_CONF, 0x02);
	px4_usleep(1000);

	const uint8_t pwr_conf_read = RegisterRead(Register::PWR_CONF);
	PX4_DEBUG("PWR_CONF readback (EXPECTING 0x02): 0x%02hhX", pwr_conf_read);


	// 3) Disable Advanced Power Save
	PX4_DEBUG("Step 3 (REPEAT): write PWR_CONF=0x02 (DISABLE APS AND leaves FSW as 1) and SLEEP FOR A LONG TIME BEFORE TRYING ANYTHING ELSE");
	RegisterWrite(Register::PWR_CONF, 0x02);
	px4_usleep(1000);
	
	const uint8_t pwr_conf_read_again = RegisterRead(Register::PWR_CONF);
	PX4_DEBUG("PWR_CONF readback (REPEAT) (EXPECTING 0x02): 0x%02hhX", pwr_conf_read_again);

	// 4) Put INIT engine into config-write mode
	PX4_DEBUG("Step 4: write INIT_CTRL=0x00 (config-write mode)");
	RegisterWrite(Register::INIT_CTRL, 0x00);
	const uint8_t init_ctrl_0 = RegisterRead(Register::INIT_CTRL);
	PX4_DEBUG("INIT_CTRL readback after 0x00: 0x%02hhX", init_ctrl_0);

	// 5) Upload in chunks (exactly like your apps driver)
	static constexpr uint16_t CHUNK = 2; // even, safe
	int res = PX4_OK;

	PX4_DEBUG("Step 5: starting config upload, len=%u bytes, CHUNK=%u",
	          (unsigned)bmi270_config_file_len, (unsigned)CHUNK);

	for (uint32_t i = 0; i < bmi270_config_file_len; i += CHUNK) {

		uint16_t wlen = (uint16_t)((bmi270_config_file_len - i < CHUNK) ? (bmi270_config_file_len - i) : CHUNK);
		if (wlen & 1) { wlen -= 1; } // keep even length

		if (wlen == 0) {
			PX4_DEBUG("CFG: wlen == 0, breaking at i=%u", (unsigned)i);
			break;
		}

		// Address packing: low nibble first, then upper bits (word address = i/2)
		const uint16_t word_addr = (uint16_t)(i / 2);
		uint8_t addr_bytes[2];
		addr_bytes[0] = (uint8_t)(word_addr & 0x0F); // low 4 bits
		addr_bytes[1] = (uint8_t)(word_addr >> 4);   // upper bits

		PX4_DEBUG("CFG: chunk i=%u wlen=%u word_addr=%u (addr0=0x%02hhX addr1=0x%02hhX)",
		          (unsigned)i, (unsigned)wlen, (unsigned)word_addr,
		          addr_bytes[0], addr_bytes[1]);

		// Write INIT_ADDR_0 + INIT_ADDR_1 in one burst starting at INIT_ADDR_0
		uint8_t tx_addr[3];
		tx_addr[0] = (((uint8_t)Register::INIT_ADDR_0) & 0x7F); // clear read/write bit
		tx_addr[1] = addr_bytes[0];
		tx_addr[2] = addr_bytes[1];

		uint8_t dummy_rx[sizeof(tx_addr)];
		res = transfer(tx_addr, dummy_rx, sizeof(tx_addr));

		if (res != PX4_OK) {
			PX4_DEBUG("CFG ERROR: INIT_ADDR write failed @i=%u, res=%d", (unsigned)i, res);
			break;
		}

		// Optional readback of INIT_ADDR_0/1 (more SPI traffic but useful)
		const uint8_t init_addr0_rb = RegisterRead(Register::INIT_ADDR_0);
		const uint8_t init_addr1_rb = RegisterRead(Register::INIT_ADDR_1);
		PX4_DEBUG("CFG: INIT_ADDR readback: [0]=0x%02hhX [1]=0x%02hhX",
		          init_addr0_rb, init_addr1_rb);

		px4_usleep(100);

		// Prepare data write
		uint8_t tx_data[1 + CHUNK];
		tx_data[0] = (((uint8_t)Register::INIT_DATA) & 0x7F);   // clear read/write bit
		memcpy(&tx_data[1], &bmi270_config_file[i], wlen);

		// Print first few data bytes for sanity
		uint8_t d0 = (wlen > 0) ? tx_data[1] : 0;
		uint8_t d1 = (wlen > 1) ? tx_data[2] : 0;
		PX4_DEBUG("CFG: DATA[0..1] @i=%u: %02hhX %02hhX",
				(unsigned)i, d0, d1);

		uint8_t dummy_rx2[1 + CHUNK];
		res = transfer(tx_data, dummy_rx2, (size_t)(1 + wlen));

		if (res != PX4_OK) {
			PX4_DEBUG("CFG ERROR: INIT_DATA write failed @i=%u, res=%d", (unsigned)i, res);
			break;
		}
	}

	if (res != PX4_OK) {
		PX4_DEBUG("CFG ERROR: res != PX4_OK at end of upload, exiting");
		return false;
	}

	PX4_DEBUG("Step 5 DONE: config upload loop finished");

	// 6) Trigger load
	PX4_DEBUG("Step 6: write INIT_CTRL=0x01 (start feature engine)");
	RegisterWrite(Register::INIT_CTRL, 0x01);
	const uint8_t init_ctrl_1 = RegisterRead(Register::INIT_CTRL);
	PX4_DEBUG("INIT_CTRL readback after 0x01: 0x%02hhX", init_ctrl_1);

	// 7) Poll INTERNAL_STATUS until 0x01 or timeout
	uint8_t istat = 0;
	const int max_tries = 200; // 1 ms each ~ 200 ms
	int tries = 0;

	PX4_DEBUG("Step 7: polling INTERNAL_STATUS for 0x01 (max_tries=%d)", max_tries);

	do {
		px4_usleep(1000);
		istat = RegisterRead(Register::INTERNAL_STATUS);
		tries++;

		// Log every 20 tries so we see some progress but not crazy spam
		if ((tries % 20) == 0 || istat == 0x01) {
			PX4_DEBUG("INTERNAL_STATUS poll: try %d -> 0x%02hhX", tries, istat);
		}

	} while (istat != 0x01 && tries < max_tries);

	PX4_DEBUG("INTERNAL_STATUS after load: 0x%02hhX (tries=%d)", istat, tries);

	if (istat != 0x01) {
		// Extended debug on failure
		uint8_t ierr  = RegisterRead(Register::INTERNAL_ERR);
		uint8_t emsk  = RegisterRead(Register::ERR_REG_MSK);
		uint8_t ictrl = RegisterRead(Register::INIT_CTRL);
		uint8_t err_reg = RegisterRead(Register::ERR_REG);

		PX4_DEBUG("Config load FAILED.");
		PX4_DEBUG("  INTERNAL_ERR = 0x%02hhX", ierr);
		PX4_DEBUG("  ERR_REG_MSK  = 0x%02hhX", emsk);
		PX4_DEBUG("  INIT_CTRL    = 0x%02hhX", ictrl);
		PX4_DEBUG("  ERR_REG      = 0x%02hhX", err_reg);

		PX4_DEBUG("=== LoadFeatureConfigAndVerify() END (FAIL) ===");
		return false;
	}

	PX4_DEBUG("Config load SUCCESS: INTERNAL_STATUS=0x%02hhX", istat);
	PX4_DEBUG("=== LoadFeatureConfigAndVerify() END (OK) ===");
	return true;
}


void VOXL_BMI270::SetGyroScale()
{
	// data is 16 bits with 2000dps range
	const float scale = math::radians(2000.0f) / 32767.0f;
	_px4_gyro.set_scale(scale);

}

void VOXL_BMI270::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	// works out to be 2 ...
	_fifo_gyro_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

// for printing clean names for debugging 
static const char *RegName(Register r)
{
	switch (r) {
	case Register::PWR_CONF:        return "PWR_CONF";
	case Register::PWR_CTRL:        return "PWR_CTRL";
	case Register::ACC_CONF:        return "ACC_CONF";
	case Register::GYR_CONF:        return "GYR_CONF";
	case Register::ACC_RANGE:       return "ACC_RANGE";
	case Register::FIFO_WTM_0:      return "FIFO_WTM_0";
	case Register::FIFO_WTM_1:      return "FIFO_WTM_1";
	case Register::FIFO_CONFIG_0:   return "FIFO_CONFIG_0";
	case Register::FIFO_CONFIG_1:   return "FIFO_CONFIG_1";
	case Register::INT1_IO_CTRL:    return "INT1_IO_CTRL";
	case Register::INT_MAP_DATA:    return "INT_MAP_DATA";
	case Register::INTERNAL_STATUS: return "INTERNAL_STATUS";
	default:                        return "UNKNOWN";
	}
}


// when this register is set an interrupt is triggered when the FIFO reaches this many samples
void VOXL_BMI270::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_WTM: 13 bit FIFO watermark level value
	// unit of the fifo watermark is one byte
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::Data);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WTM_0) {
			// fifo_water_mark[7:0]
			r.set_bits = fifo_watermark_threshold & 0x00FF;
			r.clear_bits = ~r.set_bits;

		} else if (r.reg == Register::FIFO_WTM_1) {
			// fifo_water_mark[12:8]
			r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
			r.clear_bits = ~r.set_bits;
		}
	}
}
/*
bool VOXL_BMI270::Configure()
{

	bool success = false;

	// check internal status first as per datasheet
	uint8_t internal_status = RegisterRead(Register::INTERNAL_STATUS);
	PX4_DEBUG("Internal status register value: 0x%02hhX", internal_status);

	if ((internal_status & 1) == 1) {
		PX4_DEBUG("INTERNAL_STATUS 0x01, ready for configure");

	} else {

		PX4_DEBUG("INTERNAL_STATUS check failed, resetting");
		_state = STATE::RESET;
		ScheduleDelayed(10_ms);

	};


	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured

	SetAccelScaleAndRange();
	SetGyroScale();

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	if (success == true){
		PX4_DEBUG("STATE: configure success"); 
	}
	else {
		PX4_DEBUG("STATE: configure fail");
	}
	//success = true;
	return success;
}
*/
bool VOXL_BMI270::Configure()
{
	bool success = true;  // start optimistic

	// check internal status first as per datasheet
	uint8_t internal_status = RegisterRead(Register::INTERNAL_STATUS);
	PX4_DEBUG("Internal status register (%s 0x%02hhX): 0x%02hhX",
	          RegName(Register::INTERNAL_STATUS), (uint8_t)Register::INTERNAL_STATUS, internal_status);

	if ((internal_status & 1) == 1) {
		PX4_DEBUG("INTERNAL_STATUS 0x01, ready for configure");
	} else {
		PX4_DEBUG("INTERNAL_STATUS check failed, resetting");
		_state = STATE::RESET;
		ScheduleDelayed(10_ms);
		// continue so we can still see current reg snapshots
	}

	// program registers
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// apply scale/range that may affect subsequent checks
	SetAccelScaleAndRange();
	SetGyroScale();

	// verify (always prints PASS/FAIL per register)
	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	PX4_DEBUG("STATE: configure %s", success ? "success" : "fail");
	return success;
}

int VOXL_BMI270::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	PX4_DEBUG("actual data ready interrupt called");
	static_cast<VOXL_BMI270 *>(arg)->DataReady();
	return 0;
}

void VOXL_BMI270::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool VOXL_BMI270::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool VOXL_BMI270::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}
/*
bool VOXL_BMI270::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	PX4_DEBUG("0x%02hhX: 0x%02hhX", (uint8_t)reg_cfg.reg, reg_value);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}*/
bool VOXL_BMI270::RegisterCheck(const register_config_t &reg_cfg)
{
	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	// Bits we expect to be 1 but are 0
	const uint8_t missing_set = (uint8_t)(reg_cfg.set_bits & ~reg_value);
	// Bits we expect to be 0 but are 1
	const uint8_t unexpected_set = (uint8_t)(reg_cfg.clear_bits & reg_value);

	const bool set_ok   = (missing_set == 0);
	const bool clear_ok = (unexpected_set == 0);
	const bool ok = set_ok && clear_ok;

	// Single, always-on summary line per register
	PX4_DEBUG("CFG %s (0x%02hhX): val=0x%02hhX, want_set=0x%02hhX, want_clr=0x%02hhX -> %s",
	          RegName(reg_cfg.reg), (uint8_t)reg_cfg.reg, reg_value,
	          reg_cfg.set_bits, reg_cfg.clear_bits,
	          ok ? "PASS" : "FAIL");

	// If mismatch, add explicit masks for quick diagnosis
	if (!ok) {
		if (!set_ok) {
			PX4_DEBUG("    missing_set=0x%02hhX (bits expected=1 but are 0)", missing_set);
		}
		if (!clear_ok) {
			PX4_DEBUG("    unexpected_set=0x%02hhX (bits expected=0 but are 1)", unexpected_set);
		}
	}

	return ok;
}

uint8_t VOXL_BMI270::RegisterRead(Register reg)
{
	// 6.1.2 SPI interface of accelerometer part
	//
	// In case of read operations of the accelerometer part, the requested data
	// is not sent immediately, but instead first a dummy byte is sent, and
	// after this dummy byte the actual requested register content is transmitted.
	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	// cmd[1] dummy byte
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[2];
}

void VOXL_BMI270::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void VOXL_BMI270::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}


// Checks how many bytes are in the FIFO
uint16_t VOXL_BMI270::FIFOReadCount()
{
	CheckErrorRegister();
	PX4_DEBUG("Attempting to determine FIFO fill level");

	// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
	FIFOLengthReadBuffer buffer {};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		PX4_DEBUG("Bad transfer");
		perf_count(_bad_transfer_perf);
		return 0;
	}

	uint16_t fifo_fill_level = combine(buffer.FIFO_LENGTH_1 & 0x3F, buffer.FIFO_LENGTH_0);

	return fifo_fill_level;
}


// writes a gyro frame into the FIFO buffer the first argument points to
void VOXL_BMI270::ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *frame)
{
	const uint8_t samples = gyro->samples;

	const int16_t gyro_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t gyro_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t gyro_z = combine(frame->z_msb, frame->z_lsb);

	// Rotate from FLU to NED
	gyro->x[samples] = gyro_x;
	gyro->y[samples] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	gyro->z[samples] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	gyro->samples++;
}

// writes an accelerometer frame into the FIFO buffer the first argument points to
void VOXL_BMI270::ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *frame)
{

	const uint8_t samples = accel->samples;

	const int16_t accel_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t accel_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t accel_z = combine(frame->z_msb, frame->z_lsb);

	// Rotate from FLU to NED
	accel->x[samples] = accel_x;
	accel->y[samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
	accel->z[samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	accel->samples++;
}


bool VOXL_BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{

	uint8_t err = RegisterRead(Register::ERR_REG);

	if ((err >> 6 & 1) == 1) {
		FIFOReset();
		return false;
	}

	FIFOReadBuffer buffer{};

	// Reads from the FIFO_DATA register as much data as is available,
	// plus 2 bytes for the sent command and dummy byte.
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, fifo_bytes + 2) != PX4_OK) {
		PX4_DEBUG("buffer transfer failed");
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel_buffer{};
	accel_buffer.timestamp_sample = timestamp_sample;
	accel_buffer.dt = FIFO_SAMPLE_DT;

	sensor_gyro_fifo_s gyro_buffer{};
	gyro_buffer.timestamp_sample = timestamp_sample;
	gyro_buffer.dt = FIFO_SAMPLE_DT;

	uint8_t *data_buffer = (uint8_t *)&buffer.f[0];
	unsigned fifo_buffer_index = 0; // start of buffer

	while (fifo_buffer_index < fifo_bytes) {
		// look for header signature (first 6 bits) followed by two bits indicating the status of INT1 and INT2
		switch (data_buffer[fifo_buffer_index]) {
		case ((uint8_t)FIFO::Header::sensor_accel_frame | (uint8_t)FIFO::Header::sensor_gyro_frame): {
				// Move past header and padding for Aux
				fifo_buffer_index += 1;
				VOXL_BMI270::ProcessGyro(&gyro_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
				VOXL_BMI270::ProcessAccel(&accel_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::sensor_gyro_frame: {
				// Move past header.
				fifo_buffer_index += 1;
				VOXL_BMI270::ProcessGyro(&gyro_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::sensor_accel_frame: {
				// Move past header.
				fifo_buffer_index += 1;
				VOXL_BMI270::ProcessAccel(&accel_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::skip_frame:
			// Skip Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Skip Frame");
			fifo_buffer_index += 2;
			break;

		case (uint8_t)FIFO::Header::sensor_time_frame:
			// Sensortime Frame
			// Frame length: 4 bytes (1 byte header + 3 bytes payload)
			PX4_DEBUG("Sensortime Frame");
			fifo_buffer_index += 4;
			break;

		case (uint8_t)FIFO::Header::FIFO_input_config_frame:
			// FIFO input config Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("FIFO input config Frame");
			fifo_buffer_index += 2;
			break;

		case (uint8_t)FIFO::Header::sample_drop_frame:
			// Sample drop Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Sample drop Frame");
			fifo_buffer_index += 2;
			break;

		default:
			fifo_buffer_index++;
			break;
		}
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));


	if ((accel_buffer.samples == 0) && (gyro_buffer.samples == 0)) {

		return false;

	} else {
		if (accel_buffer.samples > 0) {
			_px4_accel.updateFIFO(accel_buffer);
		}

		if (gyro_buffer.samples > 0) {
			_px4_gyro.updateFIFO(gyro_buffer);
		}

		return true;
	}

}



void VOXL_BMI270::FIFOReset()
{
	PX4_DEBUG("Resetting FIFO...");
	perf_count(_fifo_reset_perf);

	// ACC_SOFTRESET: trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	RegisterWrite(Register::CMD, 0xB0);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);
}

void VOXL_BMI270::UpdateTemperature()
{
	uint8_t temperature_buf[4] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | DIR_READ;

	if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const uint16_t temp = temperature_buf[2] | (temperature_buf[3] << 8);

	float temperature;

	if (temp == 0x8000) {
		// invalid
		temperature = NAN;

	} else {
		constexpr float lsb = 0.001953125f; // 1/2^9
		temperature = 23.0f + (int16_t)temp * lsb;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}
