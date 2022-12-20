/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPositionLoiter.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPositionLoiter::updateInitialize()
{
	bool ret = FlightTaskManualAltitude::updateInitialize();

	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskManualPositionLoiter::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	// all requirements from altitude-mode still have to hold
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// set task specific constraint
	if (_constraints.speed_xy >= _param_mpc_vel_manual.get()) {
		_constraints.speed_xy = _param_mpc_vel_manual.get();
	}

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
	_velocity_scale = _constraints.speed_xy;

	// for position-controlled mode, we need a valid position and velocity state
	// in NE-direction
	return ret;
}

float FlightTaskManualPositionLoiter::_computeVelXYGroundDist()
{
	float max_vel_xy = _constraints.speed_xy;

	// limit speed gradually within the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		max_vel_xy = math::gradual(_dist_to_ground,
					   _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					   _param_mpc_land_vel_xy.get(), _constraints.speed_xy);
	}

	return max_vel_xy;
}

void FlightTaskManualPositionLoiter::_updateXYlock()
{
	// logic as I understand:
	// regardless of status, we want to check if the reset counter was incremented, then
	if (!PX4_ISFINITE(_position_setpoint(0))) {
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);
	}
	else {
		if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counter) {
			_position_setpoint(0) = _position(0);
			_position_setpoint(1) = _position(1);
			_reset_counter = _sub_vehicle_local_position.get().xy_reset_counter;
		}
	}
}

void FlightTaskManualPositionLoiter::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_acceleration_setpoint.setNaN(); // don't use the horizontal setpoints from FlightTaskAltitude

	_updateXYlock(); // check for position lock
}

bool FlightTaskManualPositionLoiter::update()
{
	bool ret = FlightTask::update();
	_updateConstraintsFromEstimator();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();

	return ret;
}