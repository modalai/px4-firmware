/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskFollowMe.cpp
 */

#include "FlightTaskFollowMe.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>

using namespace matrix;

namespace
{
// Strike/merge overrides FORCED while FOLLOW_ME is active and restored on exit.
// They overwrite the live (incl. saved/QGC) params on entry so the merge runs at a
// known authority, and are reverted on exit so no other flight mode is affected.
// Compile-time for now; promote to FM_STRK_* params if you want to tune them live
// without a rebuild.
//
// "CHILL" SET: detuned for a gentle, slower merge that STILL commits/dives. Trailing
// comments keep the previous aggressive value so this is trivially revertible. Pair
// with voxl-vision-hub-chill.conf on the companion (V_ref/descent reduced there too).
// FM_VEL_DAMP was RAISED (more damping => lower terminal closing speed ~|a|/DAMP);
// do NOT drop it toward 0 (the old comment notes 0 ran away to 38 m/s).
struct StrikeParam { const char *name; float value; };
const StrikeParam kStrikeParams[] = {
	{"MPC_TILTMAX_AIR",  55.0f},  // aggressive bank. MUST stay > MPC_MAN_TILT_MAX(=35): the
	                              // _updateSetpoints() rescale tan(AIR)/tan(MAN) AMPLIFIES only
	                              // when AIR>MAN; the old 30 made it 0.82 -> attenuated the dive.
	{"MPC_ACC_HOR_MAX",  20.0f},  // full horizontal accel for a committed merge
	{"MPC_ACC_DOWN_MAX", 22.0f},  // fast nose-over descent RAMP (this gates how quickly vz builds)
	{"MPC_JERK_MAX",     40.0f},  // snappy setpoint slewing -> low-lag onset on pitch AND descent
	{"MPC_Z_VEL_MAX_DN",  50.0f},  // was 15: slower descent on the stock vertical path
	{"MPC_Z_VEL_MAX_UP",  50.0f},  // was 15: calmer abort climb (still safe pull-up)
	{"FM_VEL_DAMP",       0.0f},  // was 1.2: MORE damping => lower bounded closing speed
	{"FM_VEL_MAX_DN",     50.0f},  // was 30: cap the companion's commanded descent
};
static_assert(sizeof(kStrikeParams) / sizeof(kStrikeParams[0]) == 8, "update kNumStrikeParams");
}

FlightTaskFollowMe::FlightTaskFollowMe()
{
	// The tracker, not the pilot, provides horizontal/yaw steering, so do not require
	// RC sticks for the task to stay active. The vertical axis is owned by the tracker
	// (vz) or the altitude lock; the RC throttle stick is blocked from nudging altitude
	// (see kBlockRcVerticalOverride in _scaleSticks), so whether RC is present or not the
	// vehicle holds altitude unless the tracker commands a climb/descent.
	_sticks_data_required = false;
}

FlightTaskFollowMe::~FlightTaskFollowMe()
{
	_restoreStrikeParams();   // mode exit: put the pre-strike params back
}

bool FlightTaskFollowMe::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// Start from a neutral, held state until the first valid tracker intent arrives.
	_tracker_pitch_roll.setZero();
	_tracker_yaw_rate = 0.f;
	_tracker_vz = 0.f;
	_tracker_vertical_active = false;

	// Mode entry: overwrite the live params with the aggressive strike set (restored on exit).
	_forceStrikeParams();

	return ret;
}

void FlightTaskFollowMe::_forceStrikeParams()
{
	if (_strike_params_forced) {
		return;
	}

	for (int i = 0; i < kNumStrikeParams; ++i) {
		const param_t h = param_find(kStrikeParams[i].name);

		if (h == PARAM_INVALID) {
			_saved_strike_params[i] = NAN;   // nothing to restore for this one
			continue;
		}

		param_get(h, &_saved_strike_params[i]);   // cache the live value first
		float v = kStrikeParams[i].value;
		param_set_no_notification(h, &v);         // overwrite; single notify below
	}

	param_notify_changes();   // one parameter_update -> the position controller reloads the set
	_strike_params_forced = true;
	PX4_INFO("FOLLOW_ME: forced aggressive strike params");
}

void FlightTaskFollowMe::_restoreStrikeParams()
{
	if (!_strike_params_forced) {
		return;
	}

	for (int i = 0; i < kNumStrikeParams; ++i) {
		if (!PX4_ISFINITE(_saved_strike_params[i])) {
			continue;   // wasn't found / forced at entry
		}

		const param_t h = param_find(kStrikeParams[i].name);

		if (h != PARAM_INVALID) {
			param_set_no_notification(h, &_saved_strike_params[i]);
		}
	}

	param_notify_changes();
	_strike_params_forced = false;
	PX4_INFO("FOLLOW_ME: restored pre-strike params");
}

void FlightTaskFollowMe::_evaluateTrackerSetpoint()
{
	_sub_tracker_setpoint.update();
	const tracker_setpoint_s &tracker = _sub_tracker_setpoint.get();

	const bool fresh = (tracker.timestamp != 0)
			   && (hrt_absolute_time() < tracker.timestamp + kTrackerTimeoutUs);
	const bool usable = tracker.valid && fresh
			    && PX4_ISFINITE(tracker.roll_body)
			    && PX4_ISFINITE(tracker.pitch_body)
			    && PX4_ISFINITE(tracker.yaw_rate);

	if (usable) {
		// Order is (pitch, roll) to match Sticks::getPitchRoll(); clamp to the stick range
		// so a misbehaving tracker can never exceed manual tilt authority.
		_tracker_pitch_roll = Vector2f(math::constrain(tracker.pitch_body, -1.f, 1.f),
					       math::constrain(tracker.roll_body, -1.f, 1.f));
		_tracker_yaw_rate = tracker.yaw_rate;

		// Vertical intent is optional. Clamp to the configured climb/descent limits; only
		// take over the vertical axis when the demand is non-negligible, otherwise leave it
		// to the stock altitude lock so position-hold prevents drift.
		if (PX4_ISFINITE(tracker.vz)) {
			// Honor the companion's descent at strike/merge authority (FM_VEL_MAX_DN), NOT the
			// gentle MPC_Z_VEL_MAX_DN follow clamp, so the closing dive isn't throttled.
			_tracker_vz = math::constrain(tracker.vz, -_param_mpc_z_vel_max_up.get(), _param_fm_vel_max_dn.get());
			_tracker_vertical_active = fabsf(_tracker_vz) > kVzActivateThreshold;

		} else {
			_tracker_vz = 0.f;
			_tracker_vertical_active = false;
		}

	} else {
		// Lost/stale/invalid intent -> level out horizontally and hold heading + altitude.
		_tracker_pitch_roll.setZero();
		_tracker_yaw_rate = 0.f;
		_tracker_vz = 0.f;
		_tracker_vertical_active = false;
	}
}

void FlightTaskFollowMe::_scaleSticks()
{
	_evaluateTrackerSetpoint();

	// Yaw: feed the tracker's yaw rate through the same filter/lock path as the yaw stick.
	// Convert rad/s to the normalized stick equivalent so StickYaw re-applies MPC_MAN_Y_MAX
	// and locks the heading when the intent is ~0, exactly like Altitude mode.
	const float max_yaw_rate = math::max(_param_mpc_man_y_max.get(), 1e-3f);
	const float yaw_stick = math::constrain(_tracker_yaw_rate / max_yaw_rate, -1.f, 1.f);
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, yaw_stick, _yaw,
				       _is_yaw_good_for_control, _deltatime);

	// Vertical: tracker climb/descent when actively commanded, otherwise altitude hold.
	if (_tracker_vertical_active) {
		_velocity_setpoint(2) = _tracker_vz;

	} else if (kBlockRcVerticalOverride) {
		// RC THROTTLE OVERRIDE BLOCKED: the companion (tracker vz) / altitude lock fully owns the
		// vertical axis in FOLLOW_ME. Do NOT let the throttle stick nudge climb/descent. Zero the
		// vertical velocity demand so _updateAltitudeLock() (called in _updateSetpoints) holds the
		// current altitude regardless of the throttle stick position. Set kBlockRcVerticalOverride
		// = false below to restore the stock "throttle stick nudges altitude" behavior.
		_velocity_setpoint(2) = 0.f;

	} else {
		const float vel_max_z = (_sticks.getPosition()(2) > 0.f) ? _param_mpc_z_vel_max_dn.get() :
					_param_mpc_z_vel_max_up.get();
		_velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);
	}
}

void FlightTaskFollowMe::_updateSetpoints()
{
	_updateHeadingSetpoints(); // yaw setpoint from the yawspeed set in _scaleSticks()

	// Operator-in-the-loop (OGL) roll/pitch nudge: when FM_PR_NUDGE > 0, add a bounded pilot
	// stick offset ON TOP of the tracker's autonomous (pitch, roll) steering for fine manual aim
	// corrections without leaving FOLLOW_ME. Uses the expo-shaped stick (gentle near center) so
	// small inputs are precise; scaled by the authority gain and clamped to the [-1,1] stick range
	// so the combined command never exceeds full manual tilt authority before the rescale below.
	// FM_PR_NUDGE = 0 => the sticks are ignored and steering is pure tracker.
	Vector2f pitch_roll = _tracker_pitch_roll;
	const float nudge = _param_fm_pr_nudge.get();

	if (nudge > 0.f) {
		pitch_roll += Vector2f(_sticks.getPitchRollExpo()) * nudge;   // (pitch, roll), normalized [-1,1]
		pitch_roll(0) = math::constrain(pitch_roll(0), -1.f, 1.f);
		pitch_roll(1) = math::constrain(pitch_roll(1), -1.f, 1.f);
	}

	// Horizontal acceleration from the (tracker + optional pilot-nudge) roll/pitch intent, replacing
	// the stick tilt input. StickTiltXY caps the tilt at the gentle MPC_MAN_TILT_MAX; rescale to the
	// full flight envelope MPC_TILTMAX_AIR so the companion can pitch aggressively for the approach/
	// rendezvous. Since a_xy = stick * tan(tilt) * g, multiplying by tan(air)/tan(man) lifts the
	// ceiling to MPC_TILTMAX_AIR. The position controller still hard-limits attitude at
	// MPC_TILTMAX_AIR, so set that to taste.
	Vector2f accel_xy = _stick_tilt_xy.generateAccelerationSetpoints(pitch_roll, _deltatime, _yaw,
			    _yaw_setpoint);
	const float man_tilt_tan = math::max(tanf(math::radians(_param_mpc_man_tilt_max.get())), 1e-3f);
	const float air_tilt_tan = tanf(math::radians(_param_mpc_tiltmax_air.get()));
	accel_xy *= air_tilt_tan / man_tilt_tan;

	// (E) Velocity damping. FOLLOW_ME is acceleration-controlled (cloned from Altitude mode)
	// with NO horizontal velocity feedback, so the closing dynamics are undamped -> overshoot
	// and porpoising at high aggression. Oppose the current horizontal velocity so the vehicle
	// settles at a bounded terminal speed (~|a|/FM_VEL_DAMP) and oscillations are damped, instead
	// of integrating acceleration into runaway velocity. Guarded on a valid velocity estimate
	// (FOLLOW_ME requires local altitude, not necessarily horizontal position/velocity).
	if (PX4_ISFINITE(_velocity(0)) && PX4_ISFINITE(_velocity(1))) {
		const float vel_damp = _param_fm_vel_damp.get();
		accel_xy(0) -= vel_damp * _velocity(0);
		accel_xy(1) -= vel_damp * _velocity(1);
	}

	_acceleration_setpoint.xy() = accel_xy;

	if (_tracker_vertical_active) {
		// Tracker commands a climb/descent rate: velocity-control the vertical axis and release
		// the position lock. The velocity setpoint was set in _scaleSticks(); LNDMC_ALT_MAX is
		// still enforced downstream in FlightModeManager::limitAltitude().
		_position_setpoint(2) = NAN;

	} else {
		_updateAltitudeLock(); // stock baro/terrain altitude lock + position hold
		// _respectGroundSlowdown();
	}

	
}
