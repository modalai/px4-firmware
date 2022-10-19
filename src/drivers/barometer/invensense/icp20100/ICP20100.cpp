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
	uint8_t int_status;
	write_register(Register::MODE_SELECT, MODE_SELECT_BIT::STOP);
	usleep(2000);
	write_register(Register::FIFO_FILL, FIFO_FILL_BIT::FLUSH);
	write_register(Register::FIFO_CONFIG, FIFO_CONFIG_BIT::CONFIG_RESET);
	write_register(Register::INTERRUPT_MASK, INTERRUPT_MASK_BIT::INT_MASK_RESET);
	read_register(Register::INTERRUPT_STATUS, &int_status);

	if (int_status) {
		write_register(Register::INTERRUPT_STATUS, INTERRUPT_STATUS_BIT::INT_STATUS_RESET);
	}

	perf_count(_reset_perf);
	ScheduleClear();
	ScheduleDelayed(100_ms);
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
	uint8_t temp          = 0;
	uint8_t gain          = 0;
	uint8_t HFosc         = 0;
	uint8_t Rdata         = 0;
	uint8_t offset 	      = 0;
	uint8_t bootup_status = 0;

	//
	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	// Section 6.5 Step 4
	// Check the value from register regMap. OTP_STATUS2. BOOT_UP_STATUS
	//    - If 1, ICP-20100 didn’t go through power cycle after previous boot up sequence. No further
	//    initialization is required.
	read_register(Register::OTP_STATUS2, &bootup_status);

	// If 0, boot up config is not done after ICP-20100 power on. Continue to step 5
	if (bootup_status & OTP_STATUS2_BIT::BOOT_UP_STATUS) {
		PX4_INFO("Already configured");
		_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;
		return PX4_OK;
	}

	//
	// Section 6.5 Step 5
	// Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers
	// - regMap.MODE_SELECT.POWER_MODE = 1
	// - Wait 4ms;

	// Wait for MODE_SELECT register to be accessible
	check_status(Register::DEVICE_STATUS);
	read_register(Register::MODE_SELECT, &temp);
	temp |= MODE_SELECT_BIT::POWER_MODE_ACTIVE;
	write_register(Register::MODE_SELECT, temp);
	usleep(4000);

	// Section 6.5 Step 6
	//  Unlock the main registers
	// - regMap.MASTER_LOCK.LOCK = 0x1f
	write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::UNLOCK);

	// Section 6.5 Step 7
	//  Enable the OTP and the write switch
	// - regMap.OTP_CONFIG1.OTP_ENABLE = 1
	// - regMap.OTP_CONFIG1.OTP_WRITE_SWITCH = 1
	// - wait 10μs
	read_register(Register::OTP_CONFIG1, &temp);
	temp |= OTP_CONFIG1_BIT::OTP_BIT_MASK;
	write_register(Register::OTP_CONFIG1, temp);

	usleep(10);

	// Section 6.5 Step 8
	//  Toggle the OTP reset pin
	// - regMap.OTP_DBG2.RESET = 1
	// - wait 10us
	// - regMap.OTP_DBG2.RESET = 0
	// - wait 10us
	read_register(Register::OTP_DBG2, &temp);
	temp |= OTP_DBG2_BIT::DBG2_RESET;
	write_register(Register::OTP_DBG2, temp);
	usleep(10);

	temp &= ~OTP_DBG2_BIT::DBG2_RESET;
	write_register(Register::OTP_DBG2, temp);
	usleep(10);

	// Section 6.5 Step 9
	// Program redundant read
	// - regMap.OTP_MRA_LSB = 0x04
	// - regMap.OTP_MRA_MSB = 0x04
	// - regMap.OTP_MRB_LSB = 0x21
	// - regMap.OTP_MRB_MSB = 0x20
	// - regMap.OTP_MR_LSB  = 0x10
	// - regMap.OTP_MR_MSB  = 0x80
	write_register(Register::OTP_MRA_LSB, OTP_BIT::OTP_MRA_LSB);
	write_register(Register::OTP_MRA_MSB, OTP_BIT::OTP_MRA_MSB);
	write_register(Register::OTP_MRB_LSB, OTP_BIT::OTP_MRB_LSB);
	write_register(Register::OTP_MRB_MSB, OTP_BIT::OTP_MRB_MSB);
	write_register(Register::OTP_MR_LSB, OTP_BIT::OTP_MR_LSB);
	write_register(Register::OTP_MR_MSB, OTP_BIT::OTP_MR_MSB);

	// Section 6.5 Step 10
	// Write the address content and read command
	// - regMap.OTP_ADDRESS.ADDRESS = 8’hF8		//for offset
	// - regMap.OTP_COMMAND.ADDRESS = 4’h0
	// - regMap.OTP_COMMAND.COMMAND = 1		//read action
	write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::OFFSET);

	read_register(Register::OTP_COMMAND, &temp);
	temp &= ~OTP_COMMAND_BIT::ADDRESS;
	write_register(Register::OTP_COMMAND, temp);

	temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
	temp |= OTP_COMMAND_BIT::COMMAND;
	write_register(Register::OTP_COMMAND, temp);

	// Section 6.5 Step 11
	// Wait for the OTP read to finish
	// - Monitor regMap.OTP_STATUS.BUSY to be 0
	check_status(Register::OTP_STATUS);

	// Section 6.5 Step 12
	// Read the data from register
	// - Offset = regMap.OTP_RDATA.VALUE
	read_register(Register::OTP_RDATA, &offset);

	// Section 6.5 Step 13
	// Write the next address content and read command
	// - regMap.OTP_ADDRESS.ADDRESS = 8’hF9		//for gain
	// - regMap.OTP_COMMAND.ADDRESS = 4’h0
	// - regMap.OTP_COMMAND.COMMAND = 1		//read action
	write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::GAIN);

	read_register(Register::OTP_COMMAND, &temp);
	temp &= ~OTP_COMMAND_BIT::ADDRESS;
	write_register(Register::OTP_COMMAND, temp);

	temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
	temp |= OTP_COMMAND_BIT::COMMAND;
	write_register(Register::OTP_COMMAND, temp);

	// Section 6.5 Step 14
	// Wait for the OTP read to finish
	// - Monitor regMap.OTP_STATUS.BUSY to be 0
	check_status(Register::OTP_STATUS);

	// Section 6.5 Step 15
	// Read the data from register
	// - Gain = regMap.OTP_RDATA.VALUE
	read_register(Register::OTP_RDATA, &gain);

	// Section 6.5 Step 16
	// Write the next address content and read command
	// - regMap.OTP_ADDRESS.ADDRESS = 8’hFA			// for HFosc
	// - regMap.OTP_COMMAND.ADDRESS = 4’h0
	// - regMap.OTP_COMMAND.COMMAND = 1			// read action
	write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::HFOSC);

	read_register(Register::OTP_COMMAND, &temp);
	temp &= ~OTP_COMMAND_BIT::ADDRESS;
	write_register(Register::OTP_COMMAND, temp);

	temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
	temp |= OTP_COMMAND_BIT::COMMAND;
	write_register(Register::OTP_COMMAND, temp);

	// Section 6.5 Step 17
	// Wait for the OTP read to finish
	// - Monitor regMap.OTP_STATUS.BUSY to be 0
	check_status(Register::OTP_STATUS);

	// Section 6.5 Step 18
	// Read the data from register
	// - HFosc = regMap.OTP_RDATA.VALUE
	read_register(Register::OTP_RDATA, &HFosc);

	// Section 6.5 Step 19
	// Disable OTP and write switch
	// - regMap.OTP_CONFIG1.OTP_ENABLE = 0;
	// - regMap.OTP_CONFIG1.OTP_WRITE_SWITCH = 0;
	// - wait 10μs;
	read_register(Register::OTP_CONFIG1, &temp);
	temp &= ~OTP_CONFIG1_BIT::OTP_BIT_MASK;
	write_register(Register::OTP_CONFIG1, temp);
	usleep(10);

	// Section 6.5 Step 20
	// Write the Offset to the main registers
	// - regMap.TRIM1_MSB.PEFE_OFFSET_TRIM = Offset[5:0]
	offset &= OFFSET_BIT_MASK;
	write_register(Register::TRIM1_MSB, offset);

	// Section 6.5 Step 21
	// Write the Gain to the main registers without touching the parameter BG_PTAT_TRIM
	// - Rdata = regMap.TRIM2_MSB
	// - Rdata[6:4] = Gain[2:0]
	// - regMap.TRIM2_MSB = Rdata
	read_register(Register::TRIM2_MSB, &Rdata);
	Rdata = (Rdata & ~RDATA_BIT_MASK) | ((gain & GAIN_BIT_MASK) << 4);
	write_register(Register::TRIM2_MSB, Rdata);

	// Section 6.5 Step 22
	// Write the HFosc trim value to the main registers
	// - regMap.TRIM2_LSB = HFosc
	HFosc &= HFOSC_BIT_MASK;
	write_register(Register::TRIM2_LSB, HFosc);

	// Section 6.5 Step 23
	// Lock the main registers
	// - regMap.MASTER_LOCK.LOCK = 0x00
	write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK);

	// Section 6.5 Step 24
	// Move to standby
	// - regMap.MODE_SELECT.POWER_MODE = 0
	read_register(Register::MODE_SELECT, &temp);
	temp &= ~MODE_SELECT_BIT::POWER_MODE_ACTIVE;
	write_register(Register::MODE_SELECT, temp);

	// Section 6.5 Step 24
	// Write bootup config status to 1 to avoid re initialization with out power cycle.
	// - regMap.OTP_STATUS2.BOOT_UP_STATUS = 1
	read_register(Register::OTP_STATUS2, &temp);
	temp |= OTP_STATUS2_BIT::BOOT_UP_STATUS;
	write_register(Register::OTP_STATUS2, temp);

	_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;

	return PX4_OK;
}

int
ICP20100::probe()
{
	uint8_t chip_id = 111;
	uint8_t strength_check = 111;
	uint8_t drive_strength = 0x03;

	// Unlock registers so we can write drive strength
	write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::UNLOCK);

	// Wait a bit for register value to update (100ms)
	usleep(100000);

	// Set drive strength to 12mA, default on reboot is 2mA
	read_register(Register::IO_DRIVE_STRENGTH, &strength_check);

	if (strength_check != drive_strength) {
		write_register(Register::IO_DRIVE_STRENGTH, drive_strength);

		// Wait a bit for register value to update (100ms)
		usleep(100000);
	}

	// Section 6.5 Step 2
	// Initialize the I2C interface by toggling the clock line a few times. The easiest way
	// to do that is by inserting a dummy I2C write transaction.
	for (int i = 0; i < 3; ++i) {
		read_register(Register::WHO_AM_I, &chip_id);
		read_register(Register::IO_DRIVE_STRENGTH, &strength_check);

		if (chip_id == Product_ID) {
			return PX4_OK;

		} else {
			// Try to write correct drive strength again...
			write_register(Register::IO_DRIVE_STRENGTH, drive_strength);

			// Wait a bit for register value to update (100ms)
			usleep(100000);
		}
	}

	return PX4_ERROR;
}

void
ICP20100::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::INIT: {
			/*
			 * From: DS-000416-ICP-20100-v1.4.pdf
			 *
			 * Rev A (Version = 0x00) needs extra initialization sequence
			 *
			 * Section 6.5 Step 3
			 * Check the value from register regMap.version:
			 *	- If 0x00 (version A), continue to step 4.
			 *	- If 0xB2 (version B), no further initialization is required.
			 *
			 * We can check if the bootup sequence needs to be run by checking the value of TRIM2_LSB register.
			 * It has a known value before (0x1E) and after being initialize (0x1D) which can be
			 * used to check the state of the sensor.
			 *
			 */
			uint8_t version = 0;
			uint8_t reboot_check = 0;

			read_register(Register::VERSION, &version);

			if (version == ICP_20100_HW_REVA_VERSION) {
				// Running the bootup sequence multiple times without power cycling will cause issues
				read_register(Register::TRIM2_LSB, &reboot_check);

				if (reboot_check != 0x1E) {
					_state = STATE::CONFIG_MEASUREMENT;

				} else {
					_state = STATE::INIT_REV_A;
				}

			} else if (version == ICP_20100_HW_REVB_VERSION) {
				PX4_INFO("HW Rev B");
				_state = STATE::CONFIG_MEASUREMENT;

			} else {
				PX4_ERR("Unkown HW revision");
				_state = STATE::ERROR;
			}

		}

		ScheduleDelayed(40_ms);

		break;

	case STATE::INIT_REV_A: {
			if (init_boot_sequence_rev_a()) {
				PX4_ERR("Failed to initialize Rev A HW");
				_state = STATE::ERROR;

			} else {
				if (_rev_a_init_state == REV_A_INIT_STATE::SUCCESS) {
					_state = STATE::CONFIG_MEASUREMENT;
				}
			}
		}

		ScheduleDelayed(40_ms);

		break;

	case STATE::CONFIG_MEASUREMENT: {
			// Wait for MODE_SELECT to be accessible
			check_status(Register::DEVICE_STATUS);

			if (Measure()) {
				_state = STATE::READ;

				if (MEASURE_MODE != (uint8_t)MODE::MODE4) {
					init_fir();
				}

				ScheduleOnInterval(_measure_interval, _measure_interval);

			} else {
				_state = STATE::ERROR;
				PX4_ERR("FAILED TO SET MEASUREMENT RATE.");
			}
		}

		break;

	case STATE::READ: {
			uint8_t  fifo_level = 0;
			uint32_t _raw_p     = 0;
			uint32_t _raw_t     = 0;
			BurstRead buffer{};

			perf_begin(_sample_perf);

			// Read FIFO and wait until it's full
			do {
				read_register(Register::FIFO_FILL, &fifo_level);

				// Wait for FIFO to fill
				if (fifo_level == FIFO_FILL_BIT::FULL) {
					break;
				}

				usleep(1000);
			} while (1);

			// Read data from OTP registers
			burst_read_register(&buffer, (uint8_t) sizeof(buffer));

			// Calculate pressure (output in Pa)
			// P = (Pout/2^17)*40kPa + 70kPa
			_raw_p = (uint32_t)(((buffer.PRESS_DATA_2 & MEASURE_BIT_MASK) << 16) | (buffer.PRESS_DATA_1 << 8) |
					    buffer.PRESS_DATA_0);
			float pressure = ((_raw_p / 131072.f) * 40000.f) + 70000.f;

			// Calculate temperature (in C)
			// T = (Tout/2^18)*65C + 25C
			_raw_t = (uint32_t)(((buffer.TEMP_DATA_2 & MEASURE_BIT_MASK) << 16) | (buffer.TEMP_DATA_1 << 8) |
					    buffer.TEMP_DATA_0);
			float temperature = ((_raw_t / 262144.f) * 65.f) + 25.f;

			// Publish Data
			sensor_baro_s sensor_baro{};
			sensor_baro.timestamp_sample = now;
			sensor_baro.device_id = get_device_id();
			sensor_baro.pressure = pressure;
			sensor_baro.temperature = temperature;
			sensor_baro.error_count = perf_event_count(_bad_transfer_perf);
			sensor_baro.timestamp = hrt_absolute_time();
			_sensor_baro_pub.publish(sensor_baro);

		}

		perf_end(_sample_perf);

		break;

	case STATE::ERROR: {
			PX4_ERR("Fatal Error");
			exit(1);
		}
		break;
	}

}

bool
ICP20100::Measure()
{

	/* From DS-000416-ICP-20100-v1.4.pdf
	* Parameter	BW(Hz)	ODR(Hz)	Pressure Noise(PARMS)	CURRENT (ua)	IIR	FIR
	* MODE 0	6.25	25		0.5		211		NO	YES
	* MODE 1	30	120		1		222		NO	YES
	* MODE 2	10	40		2.5		49		NO	YES
	* MODE 3	0.5	2		0.5		23		NO	YES
	* MODE 4	12.5	25		0.3		250		NO	NO
	*/

	switch (_mode) {
	case MODE::MODE0:
		_measure_interval = 40_ms;
		MEASURE_MODE = MODE_SELECT_BIT::MODE0;
		write_register(Register::MODE_SELECT, MEASURE_MODE);
		break;

	case MODE::MODE1:
		_measure_interval = 8_ms;
		MEASURE_MODE = MODE_SELECT_BIT::MODE1;
		write_register(Register::MODE_SELECT, MEASURE_MODE);
		break;

	case MODE::MODE2:
		_measure_interval = 25_ms;
		MEASURE_MODE = MODE_SELECT_BIT::MODE2;
		write_register(Register::MODE_SELECT, MEASURE_MODE);
		break;

	case MODE::MODE3:
		_measure_interval = 500_ms;
		MEASURE_MODE = MODE_SELECT_BIT::MODE3;
		write_register(Register::MODE_SELECT, MEASURE_MODE);
		break;

	// User defined state not implemented...
	case MODE::MODE4:
		_measure_interval = 40_ms;
		MEASURE_MODE = MODE_SELECT_BIT::MODE4;
		write_register(Register::MODE_SELECT, MEASURE_MODE);
		break;

	default:
		_measure_interval = 25_ms;
		break;
	}

	return true;
}

bool
ICP20100::init_fir()
{
	uint8_t  int_mask   = 0;
	uint8_t  fifo_level = 0;
	uint8_t  fifo_count = 14;
	BurstRead buffer{};

	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	// Section 6.3 Step 2
	// Toggle I2C interface
	write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK);
	write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::UNLOCK);

	// Section 6.3 Step 3
	// Configure the FIFO watermark high to 14 samples
	// - regMap.FIFO_CONFIG = 0xE0
	write_register(Register::FIFO_CONFIG, FIFO_CONFIG_BIT::HW_MARK);

	// Section 6.4 Step 4
	// Unmask the watermark high interrupt
	// - regMap.INTERRUPT_MASK = 0xFB
	write_register(Register::INTERRUPT_MASK, ~(INTERRUPT_MASK_BIT::INTERRUPT_MASK));

	// Section 6.4 Step 5
	// Start a measurement
	// - regMap.MODE_SELECT.MEAS_CONFIG = M (M is the selected mode) mode = 0 (25Hz)
	// - regMap.MODE_SELECT.MEAS_MODE = 1
	// - regMap.MODE_SELECT.POWER_MODE = 0
	write_register(Register::MODE_SELECT, MEASURE_MODE);

	// Section 6.4 Step 6
	usleep(1000);

	// Section 6.4 Step 7
	// Stop the measurement
	// - regMap.MODE_SELECT = 0x00
	// - wait 10us;
	write_register(Register::MODE_SELECT, MODE_SELECT_BIT::STOP);
	usleep(10);

	// Section 6.4 Step 8
	// Flush the FIFO
	// - regMap.FIFO_FILL = 0x80;
	write_register(Register::FIFO_FILL, FIFO_FILL_BIT::FLUSH);


	// Section 6.4 Step 9
	// Reconfigure the interrupt settings if required for the application and detection of measurement data
	read_register(Register::INTERRUPT_MASK, &int_mask);
	int_mask |= INTERRUPT_MASK_BIT::INTERRUPT_MASK;
	write_register(Register::INTERRUPT_MASK, int_mask);

	// Section 6.4 Step 10
	// Start a measurement
	// - regMap.MODE_SELECT.MEAS_CONFIG = M (M is the selected mode) mode = 0 (25Hz)
	// - regMap.MODE_SELECT.MEAS_MODE = 1
	// - regMap.MODE_SELECT.POWER_MODE = 0
	write_register(Register::MODE_SELECT, MEASURE_MODE);

	// Section 6.4 Step 11 & 12
	// Wait for the interrupt or use another mechanism (polling, fixed wait) to detect if measurement data is available
	// Read the data from FIFO registers
	// - Press[7:0] = regMap.PRESS_DATA_0
	// - Press[15:8] = regMap.PRESS_DATA_1
	// - Press[19:16] = regMap.PRESS_DATA_2
	// - Temp[7:0] = regMap.TEMP_DATA_0
	// - Temp[15:8] = regMap.TEMP_DATA_1
	// - Temp[19:16] = regMap.TEMP_DATA_2
	// These values get thrown away (there is a bit of ringing in the values when sensors starts measuring)
	do {
		read_register(Register::FIFO_FILL, &fifo_level);

		if ((fifo_level &= FIFO_FILL_BIT::FILL) >= fifo_count) {
			burst_read_register(&buffer, (uint8_t) sizeof(buffer));
			break;
		}

		usleep(2);
	} while (1);

	return true;
}

int
ICP20100::read_register(Register reg, uint8_t *buf)
{
	uint8_t buff[2] = { (uint8_t)(reg), 0x00 };
	return transfer(&buff[0], 2, buf, 1);
}

int
ICP20100::burst_read_register(BurstRead *buf, uint8_t size)
{
	uint8_t cmd = static_cast<uint8_t>(Register::PRESS_DATA_0);
	return transfer(&cmd, 1, (uint8_t *)buf, size);
}

int
ICP20100::write_register(Register reg, uint8_t data)
{
	uint8_t buf[2] = { (uint8_t)(reg), data};
	return transfer(&buf[0], sizeof(buf), nullptr, 0);
}

void
ICP20100::check_status(Register reg)
{
	uint8_t status;

	while (1) {
		read_register(reg, &status);

		if (reg == Register::DEVICE_STATUS) {
			if (status & DEVICE_STATUS_BIT::SYNC) {
				break;
			}

		} else {
			if (status == OTP_STATUS_BIT::NOT_BUSY) {
				break;
			}
		}

		usleep(1);
	}
}
