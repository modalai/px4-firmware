/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	void setMaxFFSpeed(const float max_ff_speed) { _max_ff_speed = max_ff_speed; }

	void setAttitudeFFFactor(const float attitude_ff_factor) { _attitude_ff_factor = attitude_ff_factor; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 * @param dt [s] time since last calculation of desired vehicle attitude setpoint
	 */
	void setAttitudeSetpointdt(const matrix::Quatf &qd, const float yawspeed_setpoint, const float dt)
	{
		if (_max_ff_speed > 0) {
			// Parameters
			const float max_time_diff = 0.1f;

			hrt_abstime time_now = hrt_absolute_time();

			// quaternion between current and last setpoint
			matrix::Quatf q_diff = _attitude_setpoint_q.inversed() * qd;
			q_diff.normalize();
			q_diff.canonicalize();
			
			// PX4_ERR("q_diff: %f, %f, %f, %f", (double)q_diff(0), (double)q_diff(1), (double)q_diff(2), (double)q_diff(3));

			float rollspeed_raw = 0;
			float pitchspeed_raw = 0;

			//only update the ff speeds if the time has been short
			if ((time_now - _last_attitude_setpoint_time)*1e-6f < max_time_diff) {
				rollspeed_raw = 2 * q_diff(1) / dt;
				pitchspeed_raw = 2 * q_diff(2) / dt;
			}			

			//rollspeed and pitchspeed are already filtered MC_MAN_TILT_TAU > 0

			_rollspeed_setpoint = math::constrain(rollspeed_raw, -_max_ff_speed, _max_ff_speed) * _attitude_ff_factor;
			_pitchspeed_setpoint = math::constrain(pitchspeed_raw, -_max_ff_speed, _max_ff_speed) * _attitude_ff_factor;

			// PX4_ERR("time diff: %f, %f, %f, %f, %f", (double)dt, (double)rollspeed_raw, (double)_rollspeed_setpoint, (double)pitchspeed_raw, (double)_pitchspeed_setpoint);

			_last_attitude_setpoint_time = time_now;
		}
		else {
			_rollspeed_setpoint = 0.0f;
			_pitchspeed_setpoint = 0.0f;
		}

		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint

	float _rollspeed_setpoint{0.f}; ///< latest known rollspeed feed-forward setpoint
	float _pitchspeed_setpoint{0.f}; ///< latest known pitchspeed feed-forward setpoint

	float _max_ff_speed{0.f};
	float _attitude_ff_factor{0.f};

	hrt_abstime _last_attitude_setpoint_time{0};
};
