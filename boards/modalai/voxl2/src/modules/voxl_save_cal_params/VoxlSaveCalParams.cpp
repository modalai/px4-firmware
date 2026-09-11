/****************************************************************************
 *
 *   Copyright (c) 2024 ModalAI, Inc. All rights reserved.
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

#include "VoxlSaveCalParams.hpp"

#include <px4_platform_common/getopt.h>

#include <algorithm>
#include <errno.h>
#include <libgen.h>
#include <string.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

using namespace std;

ModuleBase::Descriptor VoxlSaveCalParams::desc{task_spawn, custom_command, print_usage};

static bool debug = false;
static constexpr const char *DEFAULT_CALIBRATION_DIRECTORY = "/data/modalai/px4";

VoxlSaveCalParams::VoxlSaveCalParams(const char *calibration_directory) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_calibration_directory(calibration_directory)
{
	if (_calibration_directory.back() != '/') {
		_calibration_directory += '/';
	}
}

bool
VoxlSaveCalParams::init()
{
	// Create the calibration directory, including any missing parents.
	for (size_t end = _calibration_directory.find('/', 1); end != string::npos;
	     end = _calibration_directory.find('/', end + 1)) {
		const string directory = _calibration_directory.substr(0, end);

		if (mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0 && errno != EEXIST) {
			PX4_ERR("Couldn't create calibration directory %s: %s", directory.c_str(), strerror(errno));
			return false;
		}

		struct stat info {};

		if (stat(directory.c_str(), &info) != 0 || !S_ISDIR(info.st_mode)) {
			PX4_ERR("Calibration path is not a directory: %s", directory.c_str());
			return false;
		}
	}

	if (!_parameter_primary_set_value_request_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
VoxlSaveCalParams::save_calibration_parameter_to_file(const char *name, param_type_t type,
		param_value_u value)
{
	// If the parameter being set is a calibration parameter then save it out to
	// a separate calibration file so that they can be preserved and reloaded
	// after system updates
	const char *parameter_file = param_get_default_file();
	// basename() may modify its input, so use a local copy of the parameter path.
	string parameter_file_path = parameter_file ? parameter_file : "parameters";
	string cal_file_name = _calibration_directory + basename(parameter_file_path.data());
	string cal_file_append;
	string param_name(name);
	string cal_strings[] = {"CAL_GYRO", "CAL_MAG", "CAL_BARO", "CAL_ACC"};

	for (auto i : cal_strings) {
		// Check to see if the parameter is one of the desired calibration parameters
		if (param_name.substr(0, i.size()) == i) {
			// Keep the standard parameters file basename with the calibration type
			// appended, but store it in the configured calibration directory.
			cal_file_append = i.substr(3, i.size());
			// Make sure it is lowercase
			transform(cal_file_append.begin(), cal_file_append.end(), cal_file_append.begin(), ::tolower);
			// And add a cal file extension
			cal_file_append += ".cal";
			break;
		}
	}

	// Check for level horizon calibration parameters
	if (cal_file_append.empty() &&
	    (param_name == "SENS_BOARD_X_OFF" || param_name == "SENS_BOARD_Y_OFF")) {
		cal_file_append = "_level.cal";
	}

	// Check for RC calibration parameters
	if (cal_file_append.empty() && name[0] == 'R' && name[1] == 'C' && isdigit(name[2])) {
		cal_file_append = "_rc.cal";
	}

	if (! cal_file_append.empty()) {
		cal_file_name += cal_file_append;

		stringstream param_data_stream;

		switch (type) {
		case PARAM_TYPE_INT32:
			param_data_stream << value.i;
			param_data_stream << "\t" << 6;
			break;

		case PARAM_TYPE_FLOAT:
			param_data_stream << value.f;
			param_data_stream << "\t" << 9;
			break;

		default:
			PX4_ERR("Calibration parameter must be either int or float");
			break;
		}

		string param_data;
		param_data += "1\t1\t";
		param_data += param_name;
		param_data += "\t";
		param_data += param_data_stream.str();

		if (debug) { PX4_INFO("Writing %s to file %s", param_data.c_str(), cal_file_name.c_str()); }

		// open a file in write (append) mode.
		ofstream cal_file;
		cal_file.open(cal_file_name, ios_base::app);

		if (cal_file) {
			cal_file << param_data << endl;
			cal_file.close();

		} else {
			PX4_ERR("Couldn't open %s for writing calibration value", cal_file_name.c_str());
		}
	}
}

void
VoxlSaveCalParams::Run()
{
	if (should_exit()) {
		_parameter_primary_set_value_request_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	parameter_set_value_request_s req;

	if (_parameter_primary_set_value_request_sub.copy(&req)) {
		// PX4_INFO("Got set value request in autosave module");

		// debug_counters.set_value_received++;

		param_t param = req.parameter_index;
		param_value_u value;
		value.i = 0;
		value.f = 0.0f;

		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			param_set_no_remote_update(param, (const void *) &req.int_value, true);
			value.i = req.int_value;
			break;

		case PARAM_TYPE_FLOAT:
			param_set_no_remote_update(param, (const void *) &req.float_value, true);
			value.f = req.float_value;
			break;

		default:
			PX4_ERR("Parameter must be either int or float");
			break;
		}

		save_calibration_parameter_to_file(param_name(param), param_type(param), value);
	}
}

int VoxlSaveCalParams::task_spawn(int argc, char *argv[])
{
	const char *calibration_directory = DEFAULT_CALIBRATION_DIRECTORY;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			calibration_directory = myoptarg;
			break;

		default:
			print_usage("invalid arguments");
			return PX4_ERROR;
		}
	}

	if (myoptind < argc || calibration_directory == nullptr || calibration_directory[0] == '\0') {
		print_usage("expected -d <directory> with a non-empty path");
		return PX4_ERROR;
	}

	VoxlSaveCalParams *instance = new VoxlSaveCalParams(calibration_directory);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int VoxlSaveCalParams::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VoxlSaveCalParams::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements autosaving of calibration parameters on VOXL2 platform.
Calibration files are stored in /data/modalai/px4 by default. Use -d to select
another directory. Missing directories are created at startup. Filenames retain
the parameter file basename, for example parameters_gyro.cal.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("voxl_save_cal_params", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DEFAULT_CALIBRATION_DIRECTORY, "<directory>", "Calibration file directory", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int voxl_save_cal_params_main(int argc, char *argv[])
{
	return ModuleBase::main(VoxlSaveCalParams::desc, argc, argv);
}
