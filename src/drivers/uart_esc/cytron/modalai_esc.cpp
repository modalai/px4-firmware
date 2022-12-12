/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/getopt.h>

#include "modalai_esc.hpp"
#include "modalai_esc_serial.hpp"
#include "qc_esc_packet.h"
#include "qc_esc_packet_types.h"

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"

#ifdef __PX4_QURT
#define MODALAI_ESC_DEFAULT_PORT 	"2"
#else
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"
#endif

#define LEFT_OUT 1
#define RIGHT_OUT 0

const char *_device;
static double last_time;
static uint8_t pkt[3];

ModalaiEsc::ModalaiEsc() :
	CDev(MODALAI_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_device = MODALAI_ESC_DEFAULT_PORT;

	_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);
}

ModalaiEsc::~ModalaiEsc()
{
	_outputs_on = false;

	if (_uart_port) {
		_uart_port->uart_close();
		_uart_port = nullptr;
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_output_update_perf);
}

int ModalaiEsc::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(MODALAI_ESC_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* Getting initial parameter values */
	ret = update_params();

	if (ret != OK) {
		return ret;
	}

	_uart_port = new ModalaiEscSerial();
	memset(&_esc_chans, 0x00, sizeof(_esc_chans));

	ScheduleNow();

	last_time = hrt_absolute_time();

	return 0;
}

int ModalaiEsc::load_params(uart_esc_params_t *params, ch_assign_t *map)
{
	int ret = PX4_OK;

	param_get(param_find("UART_CYT_CONFIG"),  &params->config);
	param_get(param_find("UART_CYT_BAUD"),    &params->baud_rate);
	param_get(param_find("UART_CYT_MOTOR1"),  &params->motor_map[0]);
	param_get(param_find("UART_CYT_MOTOR2"),  &params->motor_map[1]);
	param_get(param_find("UART_CYT_MOTOR3"),  &params->motor_map[2]);
	param_get(param_find("UART_CYT_MOTOR4"),  &params->motor_map[3]);
	param_get(param_find("UART_CYT_RPM_MIN"), &params->rpm_min);
	param_get(param_find("UART_CYT_RPM_MAX"), &params->rpm_max);

	if (params->rpm_min >= params->rpm_max) {
		PX4_ERR("Invalid parameter UART_CYT_RPM_MIN.  Please verify parameters.");
		params->rpm_min = 0;
		ret = PX4_ERROR;
	}

	for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		if (params->motor_map[i] == MODALAI_ESC_OUTPUT_DISABLED ||
		    params->motor_map[i] < -(MODALAI_ESC_OUTPUT_CHANNELS) ||
		    params->motor_map[i] > MODALAI_ESC_OUTPUT_CHANNELS) {
			PX4_ERR("Invalid parameter UART_CYT_MOTORX.  Please verify parameters.");
			params->motor_map[i] = 0;
			ret = PX4_ERROR;
		}

		/* Can map -4 to 4, 0 being disabled.  Negative represents reverse direction */
		map[i].number = abs(params->motor_map[i]);
		map[i].direction = (params->motor_map[i] > 0) ? 1 : -1;
	}

	return ret;
}

int ModalaiEsc::task_spawn(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_device = argv[myoptind];
			break;

		default:
			break;
		}
	}

	ModalaiEsc *instance = new ModalaiEsc();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ModalaiEsc::readResponse(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port->uart_read(_current_cmd.buf, sizeof(_current_cmd.buf));

	if (res > 0) {
		if (parseResponse(_current_cmd.buf, res) < 0) {
			PX4_ERR("Error parsing response");
			return -1;
		}

	} else {
		PX4_ERR("Read error: %i", res);
		return -1;
	}

	_current_cmd.response = false;

	return 0;
}

int ModalaiEsc::parseResponse(uint8_t *buf, uint8_t len)
{
	if (len < 4 || buf[0] != ESC_PACKET_HEADER) {
		return -1;
	}

	switch (buf[2]) {
	case ESC_PACKET_TYPE_VERSION_RESPONSE:
		if (len != sizeof(QC_ESC_VERSION_INFO)) {
			return -1;
		} else {
			QC_ESC_VERSION_INFO ver;
			memcpy(&ver, buf, len);
			PX4_INFO("ESC ID: %i", ver.id);
			PX4_INFO("HW Version: %i", ver.hw_version);
			PX4_INFO("SW Version: %i", ver.sw_version);
			PX4_INFO("Unique ID: %i", ver.unique_id);
		}

		break;

	case ESC_PACKET_TYPE_FB_RESPONSE:
    {
        // The extended feedback message and the feedback message use the same
        // message id so they need to be differentiated by message length.
        bool extended_fb = false;
		if (len == sizeof(QC_ESC_EXTENDED_FB_RESPONSE)) {
            extended_fb = true;
		} else if (len != (sizeof(QC_ESC_EXTENDED_FB_RESPONSE) - 4)) {
            // PX4_ERR("Got feedback response with invalid length %d", len);
			return -1;
		}

		QC_ESC_EXTENDED_FB_RESPONSE fb;
		memcpy(&fb, buf, len);
		uint8_t id = (fb.state & 0xF0) >> 4;

		if (id < MODALAI_ESC_OUTPUT_CHANNELS) {
			_esc_chans[id].rate_meas = fb.rpm;
			_esc_chans[id].state = fb.state & 0x0F;
			_esc_chans[id].cmd_counter = fb.cmd_counter;
			_esc_chans[id].power = fb.power;
			_esc_chans[id].voltage = fb.voltage * 0.001;

            if (extended_fb) {
				_esc_chans[id].current = fb.current * 0.008;
				_esc_chans[id].temperature = fb.temperature * 0.01;
                // PX4_INFO("EXT FB: id: %u, rpm: %u, state: %u, count: %u, pwr: %d, volts: %f, current %f, temperature %f",
                //           id, _esc_chans[id].rate_meas, _esc_chans[id].state,
                //           _esc_chans[id].cmd_counter, _esc_chans[id].power,
                //           _esc_chans[id].voltage, _esc_chans[id].current, _esc_chans[id].temperature);
            // } else {
            //     PX4_INFO("FB: id: %u, rpm: %u, state: %u, count: %u, pwr: %d, volts: %f",
            //               id, _esc_chans[id].rate_meas, _esc_chans[id].state,
            //               _esc_chans[id].cmd_counter, _esc_chans[id].power,
            //               _esc_chans[id].voltage);
            }

            // if (id == MODALAI_ESC_OUTPUT_CHANNELS - 1) {
            //     PX4_INFO("FB  %u %u %u %u", _esc_chans[0].rate_meas, _esc_chans[1].rate_meas, _esc_chans[2].rate_meas, _esc_chans[3].rate_meas);
            // }
		} else {
            PX4_ERR("Invalid ESC id %d in feedback packet", id);
        }
		break;
    }
	default:
		return -1;
	}

	return 0;
}

int ModalaiEsc::sendCommandThreadSafe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}



int ModalaiEsc::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	PX4_INFO("custom_command received");

	Command cmd;
	uint8_t esc_id = 255;
	uint8_t period = 0;
	uint8_t duration = 0;
	uint8_t power = 0;
	uint16_t led_mask = 0;
	int16_t rate = 0;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[argc - 1];

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalaiEsc::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;

	}

	while ((ch = px4_getopt(argc, argv, "i:p:d:v:l:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			esc_id = atoi(myoptarg);
			break;

		case 'p':
			period = atoi(myoptarg);
			break;

		case 'd':
			duration = atoi(myoptarg);
			break;

		case 'v':
			power = atoi(myoptarg);
			break;

		case 'l':
			led_mask = atoi(myoptarg);
			break;

		case 'r':
			rate = atoi(myoptarg);
			break;

		default:
			print_usage("Unknown command");
			return 0;
		}
	}

	if (!strcmp(verb, "reset")) {
		if (esc_id < 4) {
			PX4_INFO("Reset ESC: %i", esc_id);
			cmd.len = qc_esc_create_reset_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version")) {
		if (esc_id < 4) {
			PX4_INFO("Request version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "tone")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request tone for ESC mask: %i", esc_id);
			cmd.len = qc_esc_create_sound_packet(period, duration, power, esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "led")) {
		if (led_mask <= 0x0FFF) {
			get_instance()->_led_rsc.test = true;
			get_instance()->_led_rsc.breath_en = false;
			PX4_INFO("Request LED control for ESCs with mask: %i", led_mask);

			get_instance()->_esc_chans[0].led = (led_mask & 0x0007);
			get_instance()->_esc_chans[1].led = (led_mask & 0x0038) >> 3;
			get_instance()->_esc_chans[2].led = (led_mask & 0x01C0) >> 6;
			get_instance()->_esc_chans[3].led = (led_mask & 0x0E00) >> 9;
			return 0;

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "rpm")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request RPM for ESC bit mask: %i - RPM: %i", esc_id, rate);
			int16_t rate_req[MODALAI_ESC_OUTPUT_CHANNELS];
			int16_t outputs[MODALAI_ESC_OUTPUT_CHANNELS];
			outputs[0] = (esc_id & 1) ? rate : 0;
			outputs[1] = (esc_id & 2) ? rate : 0;
			outputs[2] = (esc_id & 4) ? rate : 0;
			outputs[3] = (esc_id & 8) ? rate : 0;

			uart_esc_params_t params;
			ch_assign_t map[MODALAI_ESC_OUTPUT_CHANNELS];
			get_instance()->load_params(&params, (ch_assign_t *)&map);

			for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
				int motor_idx = map[i].number;

				if (motor_idx > 0 && motor_idx <= MODALAI_ESC_OUTPUT_CHANNELS) {
					/* user defined mapping is 1-4, array is 0-3 */
					motor_idx--;
					rate_req[i] = outputs[motor_idx] * map[i].direction;
				}
			}

			cmd.len = qc_esc_create_rpm_packet4(rate_req[0],
							    rate_req[1],
							    rate_req[2],
							    rate_req[3],
			                    (led_mask & 0x0007),
			                    (led_mask & 0x0038) >> 3,
			                    (led_mask & 0x01C0) >> 6,
			                    (led_mask & 0x0E00) >> 9,
							    cmd.buf,
							    sizeof(cmd.buf));
			cmd.response = false;
			cmd.repeats = 500;

			if (get_instance()->sendCommandThreadSafe(&cmd) == 0) {
                // Gather ESC feedback if the test was successful
            	Command fb_cmd;
        	    memset(&fb_cmd.buf, 0x00, sizeof(fb_cmd.buf));
                for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
                	fb_cmd.len = qc_esc_create_rpm_packet4_fb(rate_req[0],
        							                          rate_req[1],
        							                          rate_req[2],
        							                          rate_req[3],
        			                                          (led_mask & 0x0007),
        			                                          (led_mask & 0x0038) >> 3,
        			                                          (led_mask & 0x01C0) >> 6,
        			                                          (led_mask & 0x0E00) >> 9,
                					                          i,
                					                          fb_cmd.buf,
                					                          sizeof(fb_cmd.buf));
			        fb_cmd.response = true;
                    if (get_instance()->sendCommandThreadSafe(&fb_cmd)) {
                        PX4_ERR("Failed to execute rpm command with feedback %d", i);
                        break;
                    } else {
                        PX4_INFO("Executed rpm command with feedback %d", i);
                    }
                }
            }
            return 0;
		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "pwm")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request PWM for ESC mask: %i - PWM: %i", esc_id, rate);
			int16_t rate_req[MODALAI_ESC_OUTPUT_CHANNELS];
			int16_t outputs[MODALAI_ESC_OUTPUT_CHANNELS];
			outputs[0] = (esc_id & 1) ? rate : 0;
			outputs[1] = (esc_id & 2) ? rate : 0;
			outputs[2] = (esc_id & 4) ? rate : 0;
			outputs[3] = (esc_id & 8) ? rate : 0;

			uart_esc_params_t params;
			ch_assign_t map[MODALAI_ESC_OUTPUT_CHANNELS];
			get_instance()->load_params(&params, (ch_assign_t *)&map);

			for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
				int motor_idx = map[i].number;

				if (motor_idx > 0 && motor_idx <= MODALAI_ESC_OUTPUT_CHANNELS) {
					/* user defined mapping is 1-4, array is 0-3 */
					motor_idx--;
					rate_req[i] = outputs[motor_idx] * map[i].direction;
				}
			}

			cmd.len = qc_esc_create_pwm_packet4(rate_req[0],
							    rate_req[1],
							    rate_req[2],
							    rate_req[3],
							    0,
							    0,
							    0,
							    0,
							    cmd.buf,
							    sizeof(cmd.buf));
			cmd.response = false;
			cmd.repeats = 500;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}
	}

	return print_usage("unknown command");
}

int ModalaiEsc::update_params()
{
	int ret = PX4_ERROR;

	updateParams();
	ret = load_params(&_parameters, (ch_assign_t *)&_output_map);

	if (ret == PX4_OK) {
		_mixing_output.setAllMinValues(_parameters.rpm_min);
		_mixing_output.setAllMaxValues(_parameters.rpm_max);
	}

	return ret;
}


int ModalaiEsc::ioctl(file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("modalai_esc ioctl cmd: %d, arg: %ld", cmd, arg);

	switch (cmd) {
	case PWM_SERVO_ARM:
		PX4_INFO("PWM_SERVO_ARM");
		break;

	case PWM_SERVO_DISARM:
		PX4_INFO("PWM_SERVO_DISARM");
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

/* OutputModuleInterface */
void ModalaiEsc::mixerChanged()
{
	/*
	 * This driver is only supporting 4 channel ESC
	 */
}


void ModalaiEsc::updateLeds(vehicle_control_mode_s mode, led_control_s control)
{
	int i = 0;
	uint8_t led_mask = _led_rsc.led_mask;

	if (_led_rsc.test) {
		return;
	}

	/*
	 * TODO - this is just a simple approach to get started.
	 *
	 */
	if (mode.timestamp != _led_rsc.mode.timestamp) {
		_led_rsc.mode = mode;
	}

	if (control.timestamp != _led_rsc.control.timestamp) {
		_led_rsc.control = control;

		switch (_led_rsc.control.color) {
		case led_control_s::COLOR_RED:
			led_mask = QC_ESC_LED_RED_ON;
			break;

		case led_control_s::COLOR_GREEN:
			led_mask = QC_ESC_LED_GREEN_ON;
			break;

		case led_control_s::COLOR_BLUE:
			led_mask = QC_ESC_LED_BLUE_ON;
			break;

		case led_control_s::COLOR_WHITE:
			led_mask = QC_ESC_LED_RED_ON | QC_ESC_LED_GREEN_ON | QC_ESC_LED_BLUE_ON;
			break;

		case led_control_s::COLOR_OFF:
			led_mask = 0;
			break;
		}

		_led_rsc.breath_en = false;

		switch (_led_rsc.control.mode) {
		case led_control_s::MODE_OFF:
			break;

		case led_control_s::MODE_ON:
			break;

		case led_control_s::MODE_DISABLED:
			break;

		case led_control_s::MODE_BLINK_SLOW:
			break;

		case led_control_s::MODE_BLINK_NORMAL:
			break;

		case led_control_s::MODE_BLINK_FAST:
			break;

		case led_control_s::MODE_BREATHE:
			_led_rsc.breath_en = true;
			_led_rsc.breath_counter = 0;
			break;

		case led_control_s::MODE_FLASH:
			break;

		default:
			break;
		}

		_led_rsc.led_mask = led_mask;
	}

	if (_led_rsc.mode.flag_armed) {
		led_mask = QC_ESC_LED_BLUE_ON;

		if (_led_rsc.mode.flag_control_position_enabled) {
			led_mask = QC_ESC_LED_GREEN_ON;

		} else if (_led_rsc.mode.flag_control_offboard_enabled) {
			led_mask = QC_ESC_LED_RED_ON;
		}

		_led_rsc.led_mask = led_mask;
	}

	if (_led_rsc.breath_en) {
		/* 8 bit counter for a decent blink visual effect for
		 * 'breathing' use case
		 */
		if ((_led_rsc.breath_counter += 8) < 128) {
			led_mask = 0;
		}
	}

	for (i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
		_esc_chans[i].led = led_mask;
	}
}

static uint16_t map_range_int(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool ModalaiEsc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			       unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (num_outputs != MODALAI_ESC_OUTPUT_CHANNELS && num_outputs != 2) {
		PX4_ERR("num_outputs != MODALAI_ESC_OUTPUT_CHANNELS %d %d",num_outputs, MODALAI_ESC_OUTPUT_CHANNELS );
		return false;
	}


	if (!_outputs_on || stop_motors) {
		outputs[LEFT_OUT] = 1500;
		outputs[RIGHT_OUT] = 1500;
	}

	bool left_reverse = false;
	bool right_reverse = false;

	// remove center frequency to origin
	int16_t left_output = (int16_t) outputs[LEFT_OUT] - 1500;
	int16_t right_output = (int16_t) outputs[RIGHT_OUT] - 1500;

	if (left_output < 0)
	{
		left_reverse = true;
	}
	if (right_output < 0)
	{
		right_reverse = true;
	}

	uint8_t left_rpm = (uint8_t)map_range_int(abs(left_output), 0, 500, 0, 63);
	uint8_t left_rpm_cmd = 0;
	if (left_reverse)
		left_rpm_cmd |= 1UL << 6;  // set
	left_rpm_cmd |= left_rpm;  // set

	uint8_t right_rpm = (uint8_t)map_range_int(abs(right_output), 0, 500, 0, 63);
	uint8_t right_rpm_cmd = 0;
	right_rpm_cmd |= 1UL << 7;  // set
	if (right_reverse)
		right_rpm_cmd |= 1UL << 6;  // set
	right_rpm_cmd |= right_rpm;  // set

	pkt[0] = '*';
	pkt[1] = left_rpm_cmd;
	pkt[2] = right_rpm_cmd;
//	if (_outputs_on)
	{

			double cur_time =  (double)hrt_absolute_time();
			double diffOutputrate = (cur_time-last_time)*10e-6;
			if (diffOutputrate >= 0.02)
			{
				if (_uart_port->uart_write(pkt,3) != 3) {
					PX4_ERR("Failed to send packet");
					return false;
				}
				last_time = hrt_absolute_time();

			}
	}

//	uint8_t pkt1,pkt2,pkt3;
//	pkt1 = '*';
//	pkt2 = left_rpm_cmd;
//	pkt3 = right_rpm_cmd;
//
//	_uart_port->uart_write(&pkt1,1);
//	//_uart_port->uart_write(&pkt2,1);
//	_uart_port->uart_write(&pkt3,1);
//



	perf_count(_output_update_perf);

	return true;
}


void ModalaiEsc::Run()
{

	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device, B230400) == PX4_OK) {
			PX4_INFO("Cytron Opened UART ESC device @ %d", B230400);
		} else {
			PX4_ERR("Failed opening device");
			return;
		}
	}

//	static double last_time = hrt_absolute_time();
//	double cur_time =  (double)hrt_absolute_time();
//	PX4_ERR("_mixing_output called %f", 1.0/((cur_time-last_time)*10e-6));
//	last_time = cur_time;

	_mixing_output.update();

	/* update output status if armed */
	_outputs_on = _mixing_output.armed().armed;

	/* check for parameter updates */
	if (!_outputs_on && _parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	//PX4_ERR("UART ESC device output %s", _current_cmd.buf);


	vehicle_control_mode_s vehicle_control_mode{};
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);
		//updateLeds(vehicle_control_mode, _led_rsc.control);
		// noop
	}
//
//	led_control_s led_control{};
//
//	if (_led_update_sub.updated()) {
//		_led_update_sub.copy(&led_control);
//		updateLeds(_led_rsc.mode, led_control);
//	}
//
//	/* breathing requires continuous updates */
//	if (_led_rsc.breath_en) {
//		updateLeds(_led_rsc.mode, _led_rsc.control);
//	}
//
//	/* Don't process commands if outputs on */
//	if (!_outputs_on) {
//		if (_current_cmd.valid()) {
//			do {
//				if (_uart_port->uart_write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
//					if (_current_cmd.repeats == 0) {
//						_current_cmd.clear();
//					}
//
//					if (_current_cmd.response) {
//						readResponse(&_current_cmd);
//					}
//
//				} else {
//					if (_current_cmd.retries == 0) {
//						_current_cmd.clear();
//						PX4_ERR("Failed to send command, errno: %i", errno);
//
//					} else {
//						_current_cmd.retries--;
//						PX4_ERR("Failed to send command, errno: %i", errno);
//					}
//				}
//
//				px4_usleep(_current_cmd.repeat_delay_us);
//			} while (_current_cmd.repeats-- > 0);
//
//		} else {
//			Command *new_cmd = _pending_cmd.load();
//
//			if (new_cmd) {
//				_current_cmd = *new_cmd;
//				_pending_cmd.store(nullptr);
//			}
//		}
//	}

	/* check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread) */
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}


int ModalaiEsc::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for...

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ todo

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("modalai_esc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Send reset request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version", "Send version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("rpm", "Closed-Loop RPM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 3, "RPM, -32,7680 to 32,768", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 800, "Duty Cycle value, 0 to 800", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("tone", "Send tone generation request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 255, "Period of sound, inverse frequency, 0-255", false);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0, 0, 255, "Duration of the sound, 0-255, 1LSB = 13ms", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', 0, 0, 100, "Power (volume) of sound, 0-100", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "Send LED control request");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 4095, "Bitmask 0x0FFF (12 bits) - ESC0 (RGB) ESC1 (RGB) ESC2 (RGB) ESC3 (RGB)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ModalaiEsc::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");

	PX4_INFO("");

	PX4_INFO("Params: UART_CYT_CONFIG: %i", _parameters.config);
	PX4_INFO("Params: UART_CYT_BAUD: %i", _parameters.baud_rate);
	PX4_INFO("Params: UART_CYT_MOTOR1: %i", _parameters.motor_map[0]);
	PX4_INFO("Params: UART_CYT_MOTOR2: %i", _parameters.motor_map[1]);
	PX4_INFO("Params: UART_CYT_MOTOR3: %i", _parameters.motor_map[2]);
	PX4_INFO("Params: UART_CYT_MOTOR4: %i", _parameters.motor_map[3]);
	PX4_INFO("Params: UART_CYT_RPM_MIN: %i", _parameters.rpm_min);
	PX4_INFO("Params: UART_CYT_RPM_MAX: %i", _parameters.rpm_max);

	PX4_INFO("");

	for( int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++){
		PX4_INFO("-- ID: %i", i);
		PX4_INFO("   State:           %i", _esc_chans[i].state);
		PX4_INFO("   Requested:       %i RPM", _esc_chans[i].rate_req);
		PX4_INFO("   Measured:        %i RPM", _esc_chans[i].rate_meas);
		PX4_INFO("   Command Counter: %i", _esc_chans[i].cmd_counter);
		PX4_INFO("   Voltage:         %f VDC", _esc_chans[i].voltage);
		PX4_INFO("");
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_output_update_perf);

	_mixing_output.printStatus();

	return 0;
}

extern "C" __EXPORT int cytron_main(int argc, char *argv[]);

int cytron_main(int argc, char *argv[])
{
	return ModalaiEsc::main(argc, argv);
}
