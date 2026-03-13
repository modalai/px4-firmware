/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Coefficient for temperature rate of change compensation for ICM42688 IMU AZ axis.
 *
 * @group Sensors
 * @min -1.0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ICM42688_DT_COMP, 0.0f);

/*
 * Shared IMU temperature calibration parameters.
 * Defined here but used by both ICM42688P and BMI270 drivers via param_find().
 */

/**
 * Enable accelerometer temperature calibration.
 *
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(IMU_TC_A_EN, 0);

/**
 * Accelerometer temperature calibration reference temperature.
 *
 * @group Sensors
 * @unit celcius
 * @min -40.0
 * @max 120.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IMU_TC_A_TREF, 30.0f);

/**
 * Accelerometer temperature slope X axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_A_SX, 0.0f);

/**
 * Accelerometer temperature slope Y axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_A_SY, 0.0f);

/**
 * Accelerometer temperature slope Z axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_A_SZ, 0.0f);

/**
 * Enable gyroscope temperature calibration.
 *
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(IMU_TC_G_EN, 0);

/**
 * Gyroscope temperature calibration reference temperature.
 *
 * @group Sensors
 * @unit celcius
 * @min -40.0
 * @max 120.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IMU_TC_G_TREF, 30.0f);

/**
 * Gyroscope temperature slope X axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_G_SX, 0.0f);

/**
 * Gyroscope temperature slope Y axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_G_SY, 0.0f);

/**
 * Gyroscope temperature slope Z axis.
 *
 * @group Sensors
 * @min -10.0
 * @max 10.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(IMU_TC_G_SZ, 0.0f);
