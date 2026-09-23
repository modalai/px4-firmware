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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void VOXL_BMI270::print_usage()
{
	PRINT_MODULE_USAGE_NAME("voxl_bmi270", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("nvm_write",
					 "Burn the CRT gyro gain trim into NVM. PERMANENT, 14 per chip for life. Requires literal arg CONFIRM");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int voxl_bmi270_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = VOXL_BMI270;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = SPI_SPEED;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_BMI270);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);

	} else if (!strcmp(verb, "nvm_write")) {
		// The BMI270 allows only 14 NVM write cycles for the LIFE of the part,
		// so require an explicit literal confirmation word. A bare
		// `voxl_bmi270 nvm_write` deliberately does nothing.
		bool confirmed = false;

		for (int i = 1; i < argc; i++) {
			if (!strcmp(argv[i], "CONFIRM")) { confirmed = true; }
		}

		if (!confirmed) {
			PX4_ERR("nvm_write is PERMANENT (14 cycles for the life of the chip).");
			PX4_ERR("Run it as:  voxl_bmi270 nvm_write CONFIRM");
			return -1;
		}

		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, true);
	}

	ThisDriver::print_usage();
	return -1;
}
