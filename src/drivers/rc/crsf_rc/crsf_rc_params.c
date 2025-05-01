/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, Inc. All rights reserved.
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
 * CRSF button 1 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON1, 0);

/**
 * CRSF button 2 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON2, 0);

/**
 * CRSF button 3 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON3, 0);

/**
 * CRSF button 3 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON4, 0);

/**
 * CRSF button 5 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON5, 0);

/**
 * CRSF button 6 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON6, 0);

/**
 * CRSF button 7 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON7, 0);

/**
 * CRSF button 8 input configuration
 *
 * Selects a button as input that will be mapped to a pwm channel output
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_BUTTON8, 0);

/**
 * CRSF button 1 output mapping
 *
 * Selects a pwm channel output for button1
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN1, 0);

/**
 * CRSF button 2 output mapping
 *
 * Selects a pwm channel output for button2
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN2, 0);

/**
 * CRSF button 3 output mapping
 *
 * Selects a pwm channel output for button3
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN3, 0);

/**
 * CRSF button 4 output mapping
 *
 * Selects a pwm channel output for button4
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN4, 0);

/**
 * CRSF button 5 output mapping
 *
 * Selects a pwm channel output for button5
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN5, 0);

/**
 * CRSF button 6 output mapping
 *
 * Selects a pwm channel output for button6
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN6, 0);

/**
 * CRSF button 7 output mapping
 *
 * Selects a pwm channel output for button7
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN7, 0);

/**
 * CRSF button 8 output mapping
 *
 * Selects a pwm channel output for button8
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMCHN8, 0);

/**
 * CRSF button 1 output value
 *
 * Selects a pwm channel output value for button1
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL1, 0);

/**
 * CRSF button 2 output value
 *
 * Selects a pwm channel output value for button2
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL2, 0);

/**
 * CRSF button 3 output value
 *
 * Selects a pwm channel output value for button3
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL3, 0);

/**
 * CRSF button 4 output value
 *
 * Selects a pwm channel output value for button4
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL4, 0);

/**
 * CRSF button 5 output value
 *
 * Selects a pwm channel output value for button5
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL5, 0);

/**
 * CRSF button 6 output value
 *
 * Selects a pwm channel output value for button6
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL6, 0);

/**
 * CRSF button 7 output value
 *
 * Selects a pwm channel output value for button7
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL7, 0);

/**
 * CRSF button 8 output value
 *
 * Selects a pwm channel output value for button8
 *
 * @reboot_required true
 *
 * @group CRSF_RC
 * @value 0 - Disabled
 * @min 0
 * @max 2000
 */
PARAM_DEFINE_INT32(CRSF_RC_PWMVAL8, 0);

