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
static  uint8_t  MEASURE_MODE;


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
	write_register(Register::MODE_SELECT,0x00);
	write_register(Register::FIFO_FILL,FIFO_FILL_BIT::FLUSH);
	write_register(Register::FIFO_CONFIG,0x00);
	write_register(Register::INTERRUPT_MASK,0xFF);
	write_register(Register::INTERRUPT_STATUS,0x00);
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

int  __attribute__((optimize("O0")))
ICP20100::init_boot_sequence_rev_a()
{
	uint8_t lsb = 0;
	uint8_t temp = 0;
	uint8_t bootup_status = 0;
	uint8_t gain = 0;
	uint8_t offset = 0;
	uint8_t HFosc = 0;
	uint8_t Rdata = 0;
	uint8_t otp_status = 1;

	uint8_t validate = 0;

	//
	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	switch (_rev_a_init_state) {
	case REV_A_INIT_STATE::NONE: {
		// Section 6.5 Step 4
		// Check the value from register regMap. OTP_STATUS2. BOOT_UP_STATUS
		//    - If 1, ICP-20100 didn’t go through power cycle after previous boot up sequence. No further
		//    initialization is required.
		if (read_register(Register::OTP_STATUS2, (uint8_t *)&bootup_status)) {
			PX4_ERR("Failed to read OTP_STATUS2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;

		}

		if (bootup_status & OTP_STATUS2_BIT::BOOT_UP_STATUS) {
			PX4_INFO("Already configured");
			_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;
			break;
		}

		//    - If 0, boot up config is not done after ICP-20100 power on. Continue to step 5

		//
		// Section 6.5 Step 5
		// Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers
		// - regMap.MODE_SELECT.POWER_MODE = 1
		// - Wait 4ms;

		temp = 0;
		/* Wait for synchronization of selected mode to the internal clock
		   domain to be finished so main registers are available to user */
		do {
			if (read_register(Register::DEVICE_STATUS, (uint8_t *)&temp)) {
				PX4_ERR("Failed to read DEVICE_STATUS");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (temp & DEVICE_STATUS_BIT::SYNC){
				break;
			}
			usleep(1);
		} while (1);


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
			usleep(100);
			read_register(Register::MODE_SELECT, (uint8_t *)&validate);	// check value
			_rev_a_init_state = REV_A_INIT_STATE::UNLOCK;
			usleep(4000);
		}
		read_register(Register::DUMMY, (uint8_t *)&validate);

	}

		break;

	case REV_A_INIT_STATE::UNLOCK: {

		// Section 6.5 Step 6
		//  Unlock the main registers
		// - regMap.MASTER_LOCK.LOCK = 0x1f
		if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::UNLOCK)) {
			PX4_ERR("Failed to write MASTER_LOCK = UNLOCK");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::MASTER_LOCK, (uint8_t *)&validate);	// check value

		if (write_register(Register::MASTER_LOCK, 0x1f)) {
			PX4_ERR("Failed to write MASTER_LOCK = UNLOCK");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::MASTER_LOCK, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 7
		//  Enable the OTP and the write switch
		// - regMap.OTP_CONFIG1.OTP_ENABLE = 1
		// - regMap.OTP_CONFIG1.OTP_WRITE_SWITCH = 1
		// - wait 10μs
		if (read_register(Register::OTP_CONFIG1, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_CONFIG1");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp |= OTP_CONFIG1_BIT::OTP_BIT_MASK;
		if (write_register(Register::OTP_CONFIG1, temp)) {
			PX4_ERR("Failed to write OTP_CONFIG1");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		usleep(10);
		read_register(Register::OTP_CONFIG1, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 8
		//  Toggle the OTP reset pin
		// - regMap.OTP_DBG2.RESET = 1
		// - wait 10us
		// - regMap.OTP_DBG2.RESET = 0
		// - wait 10us
		if (read_register(Register::OTP_DBG2, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_DBG2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp |= OTP_DBG2_BIT::RESET;
		if (write_register(Register::OTP_DBG2, temp)) {
			PX4_ERR("Failed to write OTP_DBG2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_DBG2, (uint8_t *)&validate);	// check value
		usleep(10);
		temp &= ~OTP_DBG2_BIT::RESET;
		if (write_register(Register::OTP_DBG2, temp)) {
			PX4_ERR("Failed to write OTP_DBG2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_DBG2, (uint8_t *)&validate);	// check value
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
		read_register(Register::OTP_MRA_LSB, (uint8_t *)&validate);	// check value
		if (write_register(Register::OTP_MRA_MSB, 0x04)) {
			PX4_ERR("Failed to write OTP_MRA_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_MRA_MSB, (uint8_t *)&validate);	// check value
		if (write_register(Register::OTP_MRB_LSB, 0x21)) {
			PX4_ERR("Failed to write OTP_MRB_LSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_MRB_LSB, (uint8_t *)&validate);	// check value
		if (write_register(Register::OTP_MRB_MSB, 0x20)) {
			PX4_ERR("Failed to write OTP_MRB_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_MRB_MSB, (uint8_t *)&validate);	// check value
		if (write_register(Register::OTP_MR_LSB, 0x10)) {
			PX4_ERR("Failed to write OTP_MR_LSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_MR_LSB, (uint8_t *)&validate);	// check value
		if (write_register(Register::OTP_MR_MSB, 0x80)) {
			PX4_ERR("Failed to write OTP_MR_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_MR_MSB, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 10
		// Write the address content and read command
		// - regMap.OTP_ADDRESS.ADDRESS = 8’hF8		//for offset
		// - regMap.OTP_COMMAND.ADDRESS = 4’h0
		// - regMap.OTP_COMMAND.COMMAND = 1			//read action
		if (write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::OFFSET)) {
			PX4_ERR("Failed to write OPT_ADDRESS");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_ADDRESS, (uint8_t *)&validate);	// check value

		if (read_register(Register::OTP_COMMAND, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp &= ~OTP_COMMAND_BIT::ADDRESS;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OPT_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value
		temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
		temp |= OTP_COMMAND_BIT::COMMAND;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 11
		// Wait for the OTP read to finish
		// - Monitor regMap.OTP_STATUS.BUSY to be 0
		do {
			if (read_register(Register::OTP_STATUS, (uint8_t *)&otp_status)) {
				PX4_ERR("Failed to read OTP_STATUS");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (otp_status == 0){
				break;
			}
			usleep(1);
		} while (1);

		// Section 6.5 Step 12
		// Read the data from register
		// - Offset = regMap.OTP_RDATA.VALUE
		if (read_register(Register::OTP_RDATA, (uint8_t *)&offset)) {
			PX4_ERR("Failed to read OTP_RDATA: offset");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}

		// Section 6.5 Step 13
		// Write the next address content and read command
		// - regMap.OTP_ADDRESS.ADDRESS = 8’hF9		//for gain
		// - regMap.OTP_COMMAND.ADDRESS = 4’h0
		// - regMap.OTP_COMMAND.COMMAND = 1			//read action
		if (write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::GAIN)) {
			PX4_ERR("Failed to write OPT_ADDRESS");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_ADDRESS, (uint8_t *)&validate);	// check value
		if (read_register(Register::OTP_COMMAND, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp &= ~OTP_COMMAND_BIT::ADDRESS;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value
		temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
		temp |= OTP_COMMAND_BIT::COMMAND;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 14
		// Wait for the OTP read to finish
		// - Monitor regMap.OTP_STATUS.BUSY to be 0
		do {
			if (read_register(Register::OTP_STATUS, (uint8_t *)&otp_status)) {
				PX4_ERR("Failed to read OTP_STATUS");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (otp_status == 0){
				break;
			}
			usleep(1);
		} while (1);

		// Section 6.5 Step 15
		// Read the data from register
		// - Gain = regMap.OTP_RDATA.VALUE
		if (read_register(Register::OTP_RDATA, (uint8_t *)&gain)) {
			PX4_ERR("Failed to read OTP_RDATA: gain");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}

		// Section 6.5 Step 16
		// Write the next address content and read command
		// - regMap.OTP_ADDRESS.ADDRESS = 8’hFA			// for HFosc
		// - regMap.OTP_COMMAND.ADDRESS = 4’h0
		// - regMap.OTP_COMMAND.COMMAND = 1			// read action
		if (write_register(Register::OTP_ADDRESS, OTP_ADDRESS_BIT::HFOSC)) {
			PX4_ERR("Failed to write OPT_ADDRESS");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_ADDRESS, (uint8_t *)&validate);	// check value
		if (read_register(Register::OTP_COMMAND, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp &= ~OTP_COMMAND_BIT::ADDRESS;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value
		temp &= ~OTP_COMMAND_BIT::COMMAND_BIT_MASK;
		temp |= OTP_COMMAND_BIT::COMMAND;
		if (write_register(Register::OTP_COMMAND, temp)) {
			PX4_ERR("Failed to write OTP_COMMAND");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_COMMAND, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 17
		// Wait for the OTP read to finish
		// - Monitor regMap.OTP_STATUS.BUSY to be 0
		do {
			if (read_register(Register::OTP_STATUS, (uint8_t *)&otp_status)) {
				PX4_ERR("Failed to read OTP_STATUS");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			if (otp_status == 0){
				break;
			}
			usleep(1);
		} while (1);

		// Section 6.5 Step 18
		// Read the data from register
		// - HFosc = regMap.OTP_RDATA.VALUE
		if (read_register(Register::OTP_RDATA, (uint8_t *)&HFosc)) {
			PX4_ERR("Failed to read OTP_RDATA: HFosc");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}

		// Section 6.5 Step 19
		// Disable OTP and write switch
		// - regMap.OTP_CONFIG1.OTP_ENABLE = 0;
		// - regMap.OTP_CONFIG1.OTP_WRITE_SWITCH = 0;
		// - wait 10μs;
		if (read_register(Register::OTP_RDATA, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read OTP_RDATA: HFosc");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp &= ~OTP_BIT_MASK;
		if (write_register(Register::OTP_CONFIG1, temp)) {
			PX4_ERR("Failed to write OTP_CONFIG1");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_CONFIG1, (uint8_t *)&validate);	// check value
		usleep(10);

		// Section 6.5 Step 20
		// Write the Offset to the main registers
		// - regMap.TRIM1_MSB.PEFE_OFFSET_TRIM = Offset[5:0]
		offset = (offset & (~OFFSET_BIT_MASK)) | (offset & OFFSET_BIT_MASK);
		if(write_register(Register::TRIM1_MSB, offset)) {
			PX4_ERR("Failed to write TRIM1_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::TRIM1_MSB, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 21
		// Write the Gain to the main registers without touching the parameter BG_PTAT_TRIM
		// - Rdata = regMap.TRIM2_MSB
		// - Rdata[6:4] = Gain[2:0]
		// - regMap.TRIM2_MSB = Rdata
		if(read_register(Register::TRIM2_MSB,(uint8_t *)&Rdata)){
			PX4_ERR("Failed to read TRIM2_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		// Rdata = (Rdata & (~RDATA_BIT_MASK)) | ((gain & GAIN_BIT_MASK) << 4);
		lsb = (gain & GAIN_BIT_MASK) << 4;		//get Gain[2:0] and shift into [6:4] positions
		Rdata &=~RDATA_BIT_MASK;			//set Rdata[6:4] to 0
		Rdata |= lsb;					//set Rdata[6:4] to Gain[2:0]
		if(write_register(Register::TRIM2_MSB, Rdata)) {
			PX4_ERR("Failed to write TRIM2_MSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::TRIM2_MSB, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 22
		// Write the HFosc trim value to the main registers
		// - regMap.TRIM2_LSB = HFosc
		HFosc = (HFosc & (~HFOSC_BIT_MASK)) | (HFosc & HFOSC_BIT_MASK);
		if(write_register(Register::TRIM2_LSB, HFosc)) {
			PX4_ERR("Failed to write TRIM2_LSB");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::TRIM2_LSB, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 23
		// Lock the main registers
		// - regMap.MASTER_LOCK.LOCK = 0x00
		if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK)) {
			PX4_ERR("Failed to write MASTER_LOCK = LOCK");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::MASTER_LOCK, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 24
		// Move to standby
		// - regMap.MODE_SELECT.POWER_MODE = 0
		if(read_register(Register::MODE_SELECT, (uint8_t *)&temp)){
			PX4_ERR("Failed to read MODE_SELECT");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp &=~MODE_SELECT_BIT::POWER_MODE_ACTIVE;
		if (write_register(Register::MODE_SELECT, temp)) {
			PX4_ERR("Failed to write MODE_SELECT: POWER_MODE_NORMAL");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::MODE_SELECT, (uint8_t *)&validate);	// check value

		// Section 6.5 Step 24
		// Write bootup config status to 1 to avoid re initialization with out power cycle.
		// - regMap.OTP_STATUS2.BOOT_UP_STATUS = 1
		if(read_register(Register::OTP_STATUS2,(uint8_t *)&temp)){
			PX4_ERR("Failed to read OTP_STATUS2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		temp |= OTP_STATUS2_BIT::BOOT_UP_STATUS;
		if (write_register(Register::OTP_STATUS2, temp)) {
			PX4_ERR("Failed to write OTP_STATUS2: BOOT_UP_STATUS = 1");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		read_register(Register::OTP_STATUS2, (uint8_t *)&validate);	// check value

		read_register(Register::DUMMY, &temp);

		_rev_a_init_state = REV_A_INIT_STATE::SUCCESS;
	}

	default:
		break;

	}

	return PX4_OK;
}

int __attribute__((optimize("O0")))
ICP20100::probe()
{
	uint8_t CHIP_ID = 0;
	uint8_t POWER_MODE = 0;
	uint8_t DRIVE_STRENGTH = 0;
	uint8_t MASTER_LOCK = 0;
	uint8_t OTP_CONFIG1 = 0;
	uint8_t RESULT = 0;
	uint8_t TEMP = 111;

	// Section 6.5 Step 2
	// Initialize the I2C interface by toggling the clock line a few times. The easiest way to do that is by
	// inserting a dummy I2C write transaction. You can, for example, execute the first transaction (write to
	// lock register) twice
	for (int i = 0; i < 3; i++) {
		// Checking initial values of these registers to confirm chip has correct power supply, is locked, configs, etc
		read_register(Register::MODE_SELECT, (uint8_t *)&POWER_MODE);
		read_register(Register::WHO_AM_I,(uint8_t *)&CHIP_ID);
		read_register(Register::IO_DRIVE_STRENGTH,(uint8_t *)&DRIVE_STRENGTH);
		read_register(Register::INIT,(uint8_t *)&TEMP);
		// Dummy write to help init I2C line
		write_register(Register::INIT,0xF0);
		read_register(Register::INIT,(uint8_t *)&TEMP);
		read_register(Register::MASTER_LOCK,(uint8_t *)&MASTER_LOCK);
		read_register(Register::OTP_CONFIG1,(uint8_t *)&OTP_CONFIG1);

		// Print info to Log for reading
		PX4_INFO("POWER_CONTROL: 0x%02hhX, CHIP_ID: 0x%02hhX, DRIVE_STRENGTH: 0x%02hhX", POWER_MODE, CHIP_ID, DRIVE_STRENGTH);
		PX4_INFO("MASTER_LOCK: 0x%02hhX, OTP_CONFIG1: 0x%02hhX", MASTER_LOCK, OTP_CONFIG1);

		if (CHIP_ID == Product_ID && !RESULT){
			read_register(Register::DUMMY, (uint8_t *)&POWER_MODE);
			return PX4_OK;
		}

	}

	return PX4_ERROR;
}


void  __attribute__((optimize("O0")))
ICP20100::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	static uint8_t validate = 0;

	switch (_state2) {
	case STATE2::INIT: {
		/*
		 * From: DS-000416-ICP-20100-v1.4.pdf
		 *
		 * Rev A (Version = 0x00) needs extra initialization sequence
		 *
		 * Section 6.5 Step 3
		 * Check the value from register regMap.version:
		 *	- If 0x00 (version A), continue to step 4.
		 *	- If 0xB2 (version B), no further initialization is required.
		 */
		uint8_t temp;

		//Soft Reset
		Reset();

		if (read_register(Register::VERSION, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read HW version");
			_state2 = STATE2::ERROR;

		} else {
			if (temp == ICP_20100_HW_REVA_VERSION) {
				_state2 = STATE2::INIT_REV_A;

			} else if (temp == ICP_20100_HW_REVB_VERSION) {
				PX4_INFO("HW Rev B");
				_state2 = STATE2::RESET;

			} else {
				PX4_ERR("Unkown HW revision");
				_state2 = STATE2::ERROR;
			}
		}
		}
		break;

	case STATE2::INIT_REV_A: {
			if (init_boot_sequence_rev_a()) {
				PX4_ERR("Failed to initialize Rev A HW");
				_state2 = STATE2::ERROR;

			} else {
				if(_rev_a_init_state == REV_A_INIT_STATE::SUCCESS){
					_state2 = STATE2::RESET;
				}
			}
		}
		break;

	case STATE2::RESET: {
		// check product id
		_reset_timestamp = now;
		uint8_t PROD_ID = 0;

		Reset();
		perf_count(_reset_perf);

		read_register(Register::WHO_AM_I, (uint8_t *)&PROD_ID);

		if ((uint8_t) PROD_ID == Product_ID) {
			// if reset succeeded then read otp
			_state2 = STATE2::MEASURE;
			init_fir();
			ScheduleDelayed(10_ms); // Time to coefficients are available.

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state2 = STATE2::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		ScheduleDelayed(100_ms); // Power On Reset: max 100ms

		}

		break;

	case STATE2::MEASURE: {
		uint8_t temp = 0;

		/* Wait for synchronization of selected mode to the internal clock
		   domain to be finished so main registers are available to user */
		do {
			if (read_register(Register::DEVICE_STATUS, (uint8_t *)&temp)) {
				PX4_ERR("Failed to read DEVICE_STATUS");
				_state2= STATE2::ERROR;
				break;
			}
			if (temp & DEVICE_STATUS_BIT::SYNC){
				break;
			}
			usleep(1);
		} while (1);

		if (Measure()) {
			// if configure succeeded then start measurement cycle
			_state2 = STATE2::READ;
			perf_begin(_sample_perf);
			// ScheduleOnInterval(500_ms,500_ms);
			ScheduleOnInterval(_measure_interval,_measure_interval);
		} else {
			// MEASURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Measure failed, resetting");
				_state2 = STATE2::RESET;

			} else {
				PX4_DEBUG("Measure failed, retrying");
			}

			ScheduleDelayed(_measure_interval);
		}
		}
		break;

	case STATE2::READ: {
		uint8_t failure_count = 0;
		uint32_t _raw_p = 0;
		uint32_t _raw_t = 0;
		uint8_t   temp  = 0;
		uint32_t  temp32  = 0;

		perf_end(_sample_perf);
		// Read FIFO that holds Pressure and Temp data and wait until it's full
		do{
			// break;
			if(read_register(Register::FIFO_FILL, (uint8_t *)&temp)){
				PX4_ERR("Failed to read FIFO_FILL");
				_rev_a_init_state = REV_A_INIT_STATE::ERROR;
				break;
			}
			// Wait for FIFO to fill
			if(temp < 16){
				if(temp == FIFO_FILL_BIT::FULL || temp >= 15)
					break;
				continue;
			}
			if(failure_count == 10){
				perf_count(_bad_transfer_perf);
				PX4_INFO("TIMEOUT WAITING FOR FIFO TO FILL, SIZE: %d\n", temp);
				break;
			}
			if(temp == FIFO_FILL_BIT::FULL)
				break;

			failure_count++;
			usleep(1000);
		} while (1);

		/***************** Read Pressure data *****************/

		if(read_register(Register::PRESS_DATA_0,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		_raw_p |= (uint32_t)temp;
		temp = 0;
		if(read_register(Register::PRESS_DATA_1,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		// _raw_p |= ((uint32_t)temp << 8);
		temp32  = (uint32_t)temp << 8;
		_raw_p |= temp32;
		temp = 0;
		temp32 = 0;
		if(read_register(Register::PRESS_DATA_2,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		// _raw_p |= (((uint32_t)temp & MEASURE_BIT_MASK) << 16);
		temp32 = temp & MEASURE_BIT_MASK;
		temp32 = temp32 << 16;
		_raw_p |= temp32;

		/***************** Calculate pressure (in Pa):   *****************/
		/***************** P = (Pout/2^17)*40kPa + 70kPa *****************/
		float _pressure_Pa = ((_raw_p / 131072.f) * 40000.f) + 70000.f;

		temp = 0;
		temp32 = 0;

		/***************** Read Tempertaure data *****************/

		if(read_register(Register::TEMP_DATA_0,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_0");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		_raw_t |= (uint32_t)temp;
		temp = 0;

		if(read_register(Register::TEMP_DATA_1,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_1");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		// _raw_t |= ((uint32_t)temp << 8);
		temp32  = (uint32_t)temp << 8;
		_raw_t |= temp32;

		temp = 0;
		temp32 = 0;
		if(read_register(Register::TEMP_DATA_2,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_2");
			_rev_a_init_state = REV_A_INIT_STATE::ERROR;
			break;
		}
		// _raw_t |= (((uint32_t)temp & MEASURE_BIT_MASK) << 16);
		temp32 = temp & MEASURE_BIT_MASK;
		temp32 = temp32 << 16;
		_raw_t |= temp32;

		/***************** Calculate temperature (in C): *****************/
		/***************** T = (Tout/2^18)*65C + 25C 	 *****************/
		float _temperature_C = ((_raw_t / 262144.f) * 65.f) + 25.f;

		float temperature = _temperature_C;
		float pressure = _pressure_Pa;

		printf("\n\nPressure: %.6f Pa\t", (double)pressure);
		printf("Temperature: %.6f C\n", (double)temperature);

		/***************** Publish Data *****************/
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = now;
		sensor_baro.device_id = get_device_id();
		sensor_baro.pressure = pressure;
		sensor_baro.temperature = temperature;
		sensor_baro.error_count = perf_event_count(_bad_transfer_perf);
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);

		read_register(Register::DUMMY, (uint8_t *)&validate);

		}

		break;

	case STATE2::ERROR: {
			PX4_ERR("Fatal Error");
			exit(1);
		}
		break;
	}

	ScheduleDelayed(40_ms);
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
		MEASURE_MODE = MODE_SELECT_BIT::MODE0;
		if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
			PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
			return false;
		}
		_measure_interval = 40_ms;
		break;

	case MODE::MODE1:
		MEASURE_MODE = MODE_SELECT_BIT::MODE1;
		if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
			PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
			return false;
		}
		_measure_interval = 8_ms;
		break;

	case MODE::MODE2:
		MEASURE_MODE = MODE_SELECT_BIT::MODE2;
		if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
			PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
			return false;
		}
		_measure_interval = 25_ms;
		break;

	case MODE::MODE3:
		// Start a measurement
		// - regMap.MODE_SELECT.MEAS_CONFIG = M (M is the selected mode) MODE 3 (2hz)
		// - regMap.MODE_SELECT.MEAS_MODE = 1
		// - regMap.MODE_SELECT.POWER_MODE = 0
		MEASURE_MODE = MODE_SELECT_BIT::MODE3;
		if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
			PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
			return false;
		}
		_measure_interval = 500_ms;
		break;

	// User defined state not implemented...
	case MODE::MODE4:
		MEASURE_MODE = MODE_SELECT_BIT::MODE4;
		if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
			PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
			return false;
		}
		break;
	default:
		_measure_interval = 25_ms;
		break;
	}

	return true;
}

bool  __attribute__((optimize("O0")))
ICP20100::init_fir()
{
	uint8_t  validate  = 0;
	uint32_t  temp  = 0;
	uint32_t _raw_p = 0;
	uint32_t _raw_t = 0;

	// From: DS-000416-ICP-20100-v1.4.pdf
	//
	// Section 6.3 Step 2
	// Toggle I2C interface
	if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::LOCK)) {
		PX4_ERR("Failed to toggle I2C interface");
		return false;
	}
	if (write_register(Register::MASTER_LOCK, MASTER_LOCK_BIT::UNLOCK)) {
		PX4_ERR("Failed to toggle I2C interface");
		return false;
	}

	// Section 6.3 Step 3
	// Configure the FIFO watermark high to 14 samples
	// - regMap.FIFO_CONFIG = 0xE0
	if (write_register(Register::FIFO_CONFIG, FIFO_CONFIG_BIT::HW_MARK)) {
		PX4_ERR("Failed to write FIFO_CONFIG: HIGH WATER MARK");
		return false;
	}

	// Section 6.4 Step 4
	// Unmask the watermark high interrupt
	// - regMap.INTERRUPT_MASK = 0xFB
	if (write_register(Register::INTERRUPT_MASK, ~(INTERRUPT_MASK_BIT::INTERRUPT_MASK))) {
		PX4_ERR("Failed to write INTERRUPT_MASK: ENABLE HIGH WATER MARK");
		return false;
	}

	// Section 6.4 Step 5
	// Start a measurement
	// - regMap.MODE_SELECT.MEAS_CONFIG = M (M is the selected mode) mode = 3 (2Hz)
	// - regMap.MODE_SELECT.MEAS_MODE = 1
	// - regMap.MODE_SELECT.POWER_MODE = 0
	if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
		PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
		return false;
	}

	// Section 6.4 Step 6
	// Wait for the interrupt (no interrupts so, just wait?)
	usleep(1000);

	// Section 6.4 Step 7
	// Stop the measurement
	// - regMap.MODE_SELECT = 0x00
	// - wait 10us;
	if (write_register(Register::MODE_SELECT, 0x00)) {
		PX4_ERR("Failed to write MODE_SELECT: STOP MEASUREMENT");
		return false;
	}
	usleep(10);

	// Section 6.4 Step 8
	// Flush the FIFO
	// - regMap.FIFO_FILL = 0x80;
	if (write_register(Register::FIFO_FILL, 0x80)) {
		PX4_ERR("Failed to write FIFO_FILL: FLUSH");
		return false;
	}


	// Section 6.4 Step 9
	// Reconfigure the interrupt settings if required for the application and detection of measurement data
	if (write_register(Register::INTERRUPT_MASK, INTERRUPT_MASK_BIT::INTERRUPT_MASK)) {
		PX4_ERR("Failed to write INTERRUPT_MASK: DISABLE HIGH WATER MARK");
		return false;
	}

	// Section 6.4 Step 10
	// Start a measurement
	// - regMap.MODE_SELECT.MEAS_CONFIG = M (M is the selected mode) mode = 3 (2Hz)
	// - regMap.MODE_SELECT.MEAS_MODE = 1
	// - regMap.MODE_SELECT.POWER_MODE = 0
	if (write_register(Register::MODE_SELECT, MEASURE_MODE)) {
		PX4_ERR("Failed to write MODE_SELECT: MEAS_SETTINGS");
		return false;
	}

	// Section 6.4 Step 11
	// Wait for the interrupt or use another mechanism (polling, fixed wait) to detect if measurement data is available
	usleep(1000);

	// Section 6.4 Step 11
	// Read the data from FIFO registers
	// - Press[7:0] = regMap.PRESS_DATA_0
	// - Press[15:8] = regMap.PRESS_DATA_1
	// - Press[19:16] = regMap.PRESS_DATA_2
	// - Temp[7:0] = regMap.TEMP_DATA_0
	// - Temp[15:8] = regMap.TEMP_DATA_1
	// - Temp[19:16] = regMap.TEMP_DATA_2
	// Read Pressure data
	for (int i = 0; i < 14; ++i){
		if(read_register(Register::PRESS_DATA_0,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			return false;
		}
		_raw_p |= temp;
		if(read_register(Register::PRESS_DATA_1,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			return false;
		}
		_raw_p |= ((uint32_t)temp << 8);
		// temp    = temp << 8;
		// _raw_p |= temp;
		if(read_register(Register::PRESS_DATA_2,(uint8_t *)&temp)){
			PX4_ERR("Failed to read PRESS_DATA_0");
			return false;
		}
		_raw_p |= (((uint32_t)temp & MEASURE_BIT_MASK) << 16);
		// temp = temp & MEASURE_BIT_MASK;
		// temp = temp << 16;
		// _raw_p |= temp;

		temp = 0;
		// Read Tempertaure data
		if(read_register(Register::TEMP_DATA_0,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_0");
			return false;
		}
		_raw_t |= temp;
		if(read_register(Register::TEMP_DATA_1,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_1");
			return false;
		}
		_raw_t |= temp << 8;
		// temp    = temp << 8;
		// _raw_t |= temp;
		if(read_register(Register::TEMP_DATA_2,(uint8_t *)&temp)){
			PX4_ERR("Failed to read TEMP_DATA_2");
			return false;
		}
		_raw_t |= (((uint32_t)temp & MEASURE_BIT_MASK) << 16);
		// temp &= MEASURE_BIT_MASK;
		// temp = temp << 16;
		// _raw_t |= temp;

		temp = 0;
		if (read_register(Register::FIFO_FILL, (uint8_t *)&temp)) {
			PX4_ERR("Failed to read FIFO_FILL");
			return false;
		}

		// Flush FIFO
		if (write_register(Register::FIFO_FILL, 0x80)) {
			PX4_ERR("Failed to write FIFO_FILL: FLUSH");
			return false;
		}

		if (read_register(Register::FIFO_FILL, (uint8_t *)&validate)) {
			PX4_ERR("Failed to read FIFO_FILL");
			return false;
		}

	}

	return true;
}

int
ICP20100::read_register(Register reg, uint8_t *buf)
{
	uint8_t buff[2];
	buff[0] = (uint8_t)reg;
	buff[1] = 0x00;
	return transfer(&buff[0], 2, buf, 1);
}

int __attribute__((optimize("O0")))
ICP20100::write_register(Register reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)reg;
	buf[1] = data;

	return transfer(&buf[0], sizeof(buf), nullptr, 0);
}
