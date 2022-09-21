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

/**
 * @file icp20100_registers.hpp
 *
 * icp20100 registers.
 *
 */

#pragma once

#include <cstdint>

namespace Inven_Sense_ICP20100
{
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t GAIN_BIT_MASK     = Bit2 | Bit1 | Bit0;				// Bit mask for Gain [2:0]
static constexpr uint8_t RDATA_BIT_MASK    = Bit6 | Bit5 | Bit4;				// Bit mask for RData [6:4]
static constexpr uint8_t OFFSET_BIT_MASK   = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0;		// Bit mask for Offset [5:0]
static constexpr uint8_t HFOSC_BIT_MASK	   = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0;	// Bit mask for HFSOC [6:0]
static constexpr uint8_t MEASURE_BIT_MASK  = Bit3 | Bit2 | Bit1 | Bit0;				// Bit mask for Press/Temp data

static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t  I2C_ADDRESS_DEFAULT = 0x63;

static constexpr uint8_t  Product_ID = 0x63;

static constexpr uint8_t  ICP_20100_HW_REVA_VERSION = 0x00;
static constexpr uint8_t  ICP_20100_HW_REVB_VERSION = 0xB2;

//
// From: DS-000416-ICP-20100-v1.4.pdf
//
enum class Register : uint8_t {
	TRIM1_MSB          = 0x05,
	TRIM2_LSB          = 0x06,
	TRIM2_MSB          = 0x07,
	WHO_AM_I           = 0x0C,
	IO_DRIVE_STRENGTH  = 0x0D,
	OTP_CONFIG1        = 0xAC,
	OTP_MR_LSB         = 0xAD,
	OTP_MR_MSB         = 0xAE,
	OTP_MRA_LSB        = 0xAF,
	OTP_MRA_MSB        = 0xB0,
	OTP_MRB_LSB        = 0xB1,
	OTP_MRB_MSB        = 0xB2,
	OTP_ADDRESS        = 0xB5,
	OTP_COMMAND        = 0xB6,
	OTP_RDATA          = 0xB8,
	OTP_STATUS         = 0xB9,
	OTP_DBG2           = 0xBC,
	OTP_STATUS2        = 0xBF,
	MASTER_LOCK        = 0xBE,
	MODE_SELECT        = 0xC0,
	INTERRUPT_STATUS   = 0xC1,
	INTERRUPT_MASK     = 0xC2,
	FIFO_CONFIG        = 0xC3,
	FIFO_FILL          = 0xC4,
	SPI_MODE           = 0xC5,
	PRESS_ABS_LSB      = 0xC7,
	PRESS_ABS_MSB      = 0xC8,
	PRESS_DELTA_LSB    = 0xC9,
	PRESS_DELTA_MSB    = 0xCA,
	DEVICE_STATUS      = 0xCD,
	I3C_INFO           = 0xCE,
	VERSION            = 0xD3,
	PRESS_DATA_0       = 0xFA,
	PRESS_DATA_1       = 0xFB,
	PRESS_DATA_2       = 0xFC,
	TEMP_DATA_0        = 0xFD,
	TEMP_DATA_1        = 0xFE,
	TEMP_DATA_2        = 0xFF
};

enum DEVICE_STATUS_BIT : uint8_t {
	// Make sure MODE_SELECT is accessible
	SYNC = Bit0
};

enum FIFO_CONFIG_BIT : uint8_t {
	// Set highwater mark to 14 samples for FIR initialization
	HW_MARK = Bit7 | Bit6 | Bit5,
	// Reset configs
	CONFIG_RESET = 0
};

enum FIFO_FILL_BIT : uint8_t {
	// Flush FIFO
	FLUSH = Bit7,
	// Check if FIFO full
	FULL = Bit5 | Bit4,
	// Check FIFO level
	FILL = Bit4 | Bit3 | Bit3 | Bit2 | Bit1 | Bit0
};

enum INTERRUPT_MASK_BIT : uint8_t {
	// Unmask the watermark high interrupt
	INTERRUPT_MASK = Bit2,
	// Reset interrupt mask
	INT_MASK_RESET = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit3 | Bit2 | Bit1 | Bit0
};

enum INTERRUPT_STATUS_BIT : uint8_t {
	// Reset interrupt mask
	INT_STATUS_RESET = 0
};

enum OTP_BIT : uint8_t {
	// OTP_MRA_LSB
	OTP_MRA_LSB = Bit2,
	// OTP_MRA_MSB
	OTP_MRA_MSB = Bit2,
	// OTP_MRB_LSB
	OTP_MRB_LSB = Bit5 | Bit0,
	// OTP_MRB_MSB
	OTP_MRB_MSB = Bit5,
	// OTP_MR_LSB
	OTP_MR_LSB  = Bit4,
	// OTP_MR_MSB
	OTP_MR_MSB = Bit7
};

enum OTP_STATUS2_BIT : uint8_t {
	// Boot up config status.
	BOOT_UP_STATUS = Bit0
};

enum MASTER_LOCK_BIT : uint8_t {
	// Lock or unlock main registers
	LOCK = 0x00,
	UNLOCK = 0x1F
};

enum MODE_SELECT_BIT : uint8_t {
	// Bring chip into active power mode
	POWER_MODE_ACTIVE = Bit2,

	// Stop measurement
	STOP = 0,

	/* Example for Mode0:
	* Mode0 measure config (000), Standby mode (Bit4 = 0), Duty cycle mode (Bit3 = 1),
	* Normal power mode (Bit2 = 0), Pressure first read out (Bit1 & Bit0 = 0)
	*/
	MODE0 = Bit3,			// [7:5] 000 01000
	MODE1 = Bit5 | Bit3,		// [7:5] 001 01000
	MODE2 = Bit6 | Bit3,		// [7:5] 010 01000
	MODE3 = Bit6 | Bit5 | Bit3,	// [7:5] 011 01000
	MODE4 = Bit7 | Bit3		// [7:5] 100 01000
};

enum OTP_ADDRESS_BIT : uint8_t {
	// OTP address [7:0] to read from or to write to
	OFFSET = Bit7 | Bit6 | Bit5 | Bit4 | Bit3,
	GAIN   = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit0,
	HFOSC  = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit1
};

enum OTP_COMMAND_BIT : uint8_t {
	// 6:4, OTP access command
	COMMAND_BIT_MASK = Bit6 | Bit5 | Bit4,
	COMMAND = Bit4,
	// OTP address [3:0] to read from or to write to
	ADDRESS = Bit3 | Bit2 | Bit1 | Bit0
};

enum OTP_CONFIG1_BIT : uint8_t {
	// Connect OTP VCC to VCORE and enable the OTP. This is needed for OTP write. VCORE should be 3V3 in this case
	OTP_BIT_MASK = Bit1 | Bit0
};

enum OTP_DBG2_BIT : uint8_t {
	// Reset value of the OTP port RESET
	DBG2_RESET = Bit7
};

enum OTP_STATUS_BIT : uint8_t {
	// OTP controller BUSY flag
	BUSY = Bit0,
	// OTP controller NOT BUSY flag
	NOT_BUSY = 0
};


} // namespace Inven_Sense_ICP20100
