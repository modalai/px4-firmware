/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gravity_xyz_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_gravity_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_gravity_z_innov_var_and_h.h>

#include <mathlib/mathlib.h>

void Ekf::GravityVectorFilter::reset(float tau_s, float dt_s, const Vector3f &u)
{
	tau = tau_s;
	dt = dt_s;

	// second-order Butterworth, cutoff sqrt(2) / (2 pi tau) as in VQF
	const double C = tan(sqrt(2.0) * double(dt_s) / (2.0 * double(tau_s)));
	const double D = C * C + sqrt(2.0) * C + 1.0;
	b0 = C * C / D;
	a1 = 2.0 * (C * C - 1.0) / D;
	a2 = (1.0 - sqrt(2.0) * C + C * C) / D;

	// start in steady state at the current input
	for (int i = 0; i < 3; i++) {
		s1[i] = double(u(i)) * (1.0 - b0);
		s2[i] = double(u(i)) * (b0 - a2);
	}

	initialised = true;
}

Vector3f Ekf::GravityVectorFilter::update(const Vector3f &u)
{
	Vector3f y;

	for (int i = 0; i < 3; i++) {
		const double x = double(u(i));
		const double yi = b0 * x + s1[i];
		s1[i] = 2.0 * b0 * x - a1 * yi + s2[i];
		s2[i] = b0 * x - a2 * yi;
		y(i) = float(yi);
	}

	return y;
}

void Ekf::controlGravityFusion(const imuSample &imu)
{
	// raw accelerometer reading at the delayed horizon
	const Vector3f raw_force = imu.delta_vel / imu.delta_vel_dt - _state.accel_bias;

	// EKF2_GRAV_FTAU: filter the specific force in a frame propagated by the bias-corrected
	// gyro. Gravity is constant in that frame and passes the low-pass unchanged, while other
	// accelerations average to roughly their velocity change over tau. Rotating the result back
	// with the current gyro frame gives the gravity direction in today's body axes, without
	// the lag of filtering in the body frame. The frame's starting attitude is arbitrary and
	// cancels out. The filter keeps running while fusion is gated so its history includes
	// the manoeuvre.
	Vector3f force = raw_force;

	if (_params.ekf2_grav_ftau > 0.f) {
		const float nominal_dt = math::constrain(_dt_ekf_avg, 0.001f, 0.05f);

		// restart on tilt (re)alignment, IMU gaps and parameter changes
		const bool restart = !_control_status.flags.tilt_align
				     || (imu.delta_ang_dt > 0.05f)
				     || !_grav_filt_slow.initialised
				     || (fabsf(_grav_filt_slow.tau - _params.ekf2_grav_ftau) > FLT_EPSILON)
				     || (fabsf(_grav_filt_slow.dt - nominal_dt) > 0.2f * nominal_dt);

		if (restart) {
			_grav_gyro_frame = Quatf();
			_grav_filt_slow.reset(_params.ekf2_grav_ftau, nominal_dt, raw_force);

		} else {
			// same delta angle correction as the state prediction: gyro bias and earth rate
			const Vector3f corrected_delta_ang = imu.delta_ang - getGyroBias() * imu.delta_ang_dt
							     - _R_to_earth.transpose() * _earth_rate_NED * imu.delta_ang_dt;
			_grav_gyro_frame = _grav_gyro_frame * Quatf(AxisAnglef(corrected_delta_ang));
			_grav_gyro_frame.normalize();
		}

		const Vector3f u = _grav_gyro_frame.rotateVector(raw_force);
		const Vector3f slow = _grav_filt_slow.update(u);
		force = _grav_gyro_frame.rotateVectorInverse(slow);

	} else {
		_grav_filt_slow.initialised = false;
	}

	const Vector3f measurement = force.unit();
	const float measurement_var = math::max(sq(_params.ekf2_grav_noise), sq(0.01f));

	// EKF2_GRAV_AWIN: accepted deviation of the filtered accel norm from 1 g (default 0.1)
	const float accel_window = math::constrain(_params.ekf2_grav_awin, 0.01f, 0.5f);
	const float upper_accel_limit = CONSTANTS_ONE_G * (1.f + accel_window);
	const float lower_accel_limit = CONSTANTS_ONE_G * (1.f - accel_window);
	const float accel_lpf_norm_sq = _accel_lpf.getState().norm_squared();
	const bool accel_lpf_norm_good = (accel_lpf_norm_sq > sq(lower_accel_limit))
					 && (accel_lpf_norm_sq < sq(upper_accel_limit));

	// On some airframes the accel norm check above is not enough: during aggressive
	// flight the norm dips back inside the window while the specific force is still
	// dominated by thrust. Optionally also require a low rotation rate. EKF2_GRAV_RMODE
	// selects which rate is tested - the instantaneous term blocks at the onset of
	// motion, the filtered term holds fusion off until settled, and the default (2)
	// requires both. The check is off unless EKF2_GRAV_RMAX > 0.
	bool gyro_rate_good = true;

	if (_params.ekf2_grav_rmax > 0.f) {
		const float rate_limit = math::radians(_params.ekf2_grav_rmax);
		const float rate_filtered = _gyro_lpf.getState().norm();
		const float rate_instant = (imu.delta_ang_dt > 1e-6f)
					   ? Vector3f(imu.delta_ang / imu.delta_ang_dt).norm()
					   : rate_filtered;

		switch (_params.ekf2_grav_rmode) {
		case 0: // filtered only
			gyro_rate_good = (rate_filtered < rate_limit);
			break;

		case 1: // instantaneous only
			gyro_rate_good = (rate_instant < rate_limit);
			break;

		default: // both
			gyro_rate_good = (rate_instant < rate_limit) && (rate_filtered < rate_limit);
			break;
		}
	}

	// fuse gravity observation if our overall acceleration isn't too big
	_control_status.flags.gravity_vector = (_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					       && (accel_lpf_norm_good || _control_status.flags.vehicle_at_rest)
					       && gyro_rate_good
					       && !isHorizontalAidingActive()
					       && _control_status.flags.tilt_align; // Let fake position do the initial alignment (more robust before takeoff)

	// calculate kalman gains and innovation variances
	Vector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement;
	Vector3f innovation_variance;
	const auto state_vector = _state.vector();
	VectorState H;
	sym::ComputeGravityXyzInnovVarAndHx(state_vector, P, measurement_var, &innovation_variance, &H);

	// Use a wide gate to admit the large attitude errors that accumulate while
	// this fusion is unavailable during maneuvers (|a| outside the 0.9-1.1g
	// window), allowing the filter to recover when gravity fusion resumes.
	const float innovation_gate = 4.f;

	// fill estimator aid source status
	updateAidSourceStatus(_aid_src_gravity,
			      imu.time_us,                                                 // sample timestamp
			      measurement,                                                 // observation
			      Vector3f{measurement_var, measurement_var, measurement_var}, // observation variance
			      innovation,                                                  // innovation
			      innovation_variance,                                         // innovation variance
			      innovation_gate);                                            // innovation gate

	// update the states and covariance using sequential fusion
	bool fused[3] {};

	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityYInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);

		} else if (index == 2) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityZInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);
		}

		VectorState K = P * H / _aid_src_gravity.innovation_variance[index];

		const bool accel_clipping = imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2];

		if (_control_status.flags.gravity_vector && !_aid_src_gravity.innovation_rejected && !accel_clipping) {

			fused[index] = measurementUpdate(K, H,
							 _aid_src_gravity.observation_variance[index], _aid_src_gravity.innovation[index]);
		}
	}

	if (fused[0] && fused[1] && fused[2]) {
		_aid_src_gravity.fused = true;
		_aid_src_gravity.time_last_fuse = imu.time_us;

	} else {
		_aid_src_gravity.fused = false;
	}
}
