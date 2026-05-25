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
 * The vertical axis keeps the stock Altitude-mode logic (baro/altitude lock,
 * optionally nudged by the RC throttle stick). Intended for follow-me where an
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
	virtual ~FlightTaskFollowMe() = default;

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

	uORB::SubscriptionData<tracker_setpoint_s> _sub_tracker_setpoint{ORB_ID(tracker_setpoint)};

	matrix::Vector2f _tracker_pitch_roll{0.f, 0.f}; ///< (pitch, roll) normalized [-1,1], order matches Sticks::getPitchRoll()
	float _tracker_yaw_rate{0.f};                    ///< desired yaw rate [rad/s]
	float _tracker_vz{0.f};                          ///< desired vertical velocity, NED [m/s, +down], clamped to MPC_Z_VEL_MAX_*
	bool _tracker_vertical_active{false};            ///< true when the tracker is commanding a non-negligible climb/descent

	static constexpr uint64_t kTrackerTimeoutUs = 500000; ///< intent older than this is treated as lost
	static constexpr float kVzActivateThreshold = 0.05f;  ///< [m/s] below this the vertical axis stays on stock altitude lock

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAltitude,
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max, /**< max yaw rate, used to normalize tracker yaw_rate */
					(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max, /**< manual tilt cap baked into the StickTiltXY output, divided back out here */
					(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air, /**< full flight tilt envelope; the tracker tilt is scaled up to this for aggressive approach/rendezvous */
					(ParamFloat<px4::params::FM_VEL_DAMP>) _param_fm_vel_damp /**< [1/s] horizontal velocity-damping gain (a -= FM_VEL_DAMP*v); terminal speed ~|a|/FM_VEL_DAMP */
				       )
};
