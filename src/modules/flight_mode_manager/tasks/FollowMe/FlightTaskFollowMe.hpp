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
 * @file FlightTaskFollowMe.hpp
 *
 * Flight task that behaves like manual Altitude mode but takes its horizontal
 * (roll/pitch) and yaw steering from an external tracker instead of the sticks.
 * The vertical axis follows the tracker's vz when commanded, else holds altitude
 * via the stock baro/altitude lock; the RC throttle stick is blocked from nudging
 * the vertical axis (see kBlockRcVerticalOverride). Intended for follow-me where an
 * off-board computer publishes tracker_setpoint.
 */

#pragma once

#include "FlightTaskManualAltitude.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/tracker_setpoint.h>

class FlightTaskFollowMe : public FlightTaskManualAltitude
{
public:
	FlightTaskFollowMe();
	~FlightTaskFollowMe() override;   // restores the pre-strike params (mode exit hook)

	bool activate(const trajectory_setpoint_s &last_setpoint) override;

protected:
	/** Replace the yaw/vertical stick handling: yaw from tracker intent, vertical kept on stock Altitude logic. */
	void _scaleSticks() override;

	/** Replace the horizontal tilt stick handling with the tracker roll/pitch intent. */
	void _updateSetpoints() override;

private:
	/**
	 * Latch the latest tracker intent for this control cycle.
	 * Falls back to zero intent (level out + altitude hold) when the message is
	 * missing, stale or flagged invalid, so loss of the tracker is fail-safe.
	 */
	void _evaluateTrackerSetpoint();

	/**
	 * Force the aggressive strike/merge param set on mode ENTRY (caching each live value
	 * first) and restore the cached values on EXIT. This overrides the saved/QGC params so
	 * the merge always runs at full authority, while leaving every other flight mode on the
	 * normal tuning. Restore runs from the destructor — FlightModeManager destroys the task
	 * on any mode switch or shutdown, so exit is always covered (and a reboot reloads the
	 * saved params if a restore is ever missed).
	 */
	void _forceStrikeParams();
	void _restoreStrikeParams();

	static constexpr int kNumStrikeParams = 8;
	float _saved_strike_params[kNumStrikeParams] {}; ///< live values cached at entry, written back at exit
	bool  _strike_params_forced{false};

	uORB::SubscriptionData<tracker_setpoint_s> _sub_tracker_setpoint{ORB_ID(tracker_setpoint)};

	matrix::Vector2f _tracker_pitch_roll{0.f, 0.f}; ///< (pitch, roll) normalized [-1,1], order matches Sticks::getPitchRoll()
	float _tracker_yaw_rate{0.f};                    ///< desired yaw rate [rad/s]
	float _tracker_vz{0.f};                          ///< desired vertical velocity, NED [m/s, +down], clamped to MPC_Z_VEL_MAX_*
	bool _tracker_vertical_active{false};            ///< true when the tracker is commanding a non-negligible climb/descent

	static constexpr uint64_t kTrackerTimeoutUs = 500000; ///< intent older than this is treated as lost
	static constexpr float kVzActivateThreshold = 0.05f;  ///< [m/s] below this the vertical axis stays on stock altitude lock
	static constexpr bool kBlockRcVerticalOverride = true; ///< FOLLOW_ME: ignore the RC throttle stick on the vertical axis (companion/altitude-lock owns vz). false = stock throttle-nudges-altitude behavior

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAltitude,
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max, /**< max yaw rate, used to normalize tracker yaw_rate */
					(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max, /**< manual tilt cap baked into the StickTiltXY output, divided back out here */
					(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air, /**< full flight tilt envelope; the tracker tilt is scaled up to this for aggressive approach/rendezvous */
					(ParamFloat<px4::params::FM_VEL_DAMP>) _param_fm_vel_damp, /**< [1/s] horizontal velocity-damping gain (a -= FM_VEL_DAMP*v); terminal speed ~|a|/FM_VEL_DAMP. 0 = pure accel (max aggression, default) */
					(ParamFloat<px4::params::FM_VEL_MAX_DN>) _param_fm_vel_max_dn, /**< [m/s] max tracker-commanded descent; replaces the gentle MPC_Z_VEL_MAX_DN clamp on the strike/merge path */
					(ParamFloat<px4::params::FM_PR_NUDGE>) _param_fm_pr_nudge /**< [0..1] pilot roll/pitch stick nudge authority added on top of the tracker intent; 0 = pure tracker (OGL operator-in-the-loop) */
				       )
};
