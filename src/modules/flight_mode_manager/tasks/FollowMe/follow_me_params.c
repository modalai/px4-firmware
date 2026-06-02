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
 * @file follow_me_params.c
 *
 * Parameters for the tracker-driven FOLLOW_ME flight task.
 */

/**
 * FollowMe horizontal velocity damping gain
 *
 * FOLLOW_ME is acceleration-controlled (cloned from Altitude mode) with no
 * horizontal velocity feedback, so the closing dynamics are otherwise undamped
 * and porpoise at high aggression. The horizontal acceleration setpoint is
 * reduced by this gain times the current horizontal velocity, giving a bounded
 * terminal speed of ~|a| / FM_VEL_DAMP. Higher = more damped / slower; 0 = no
 * damping (pure acceleration, most aggressive, risks overshoot/porpoising).
 *
 * @unit 1/s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group FlightTaskFollowMe
 */
PARAM_DEFINE_FLOAT(FM_VEL_DAMP, 0.0f);

/**
 * FollowMe max tracker-commanded descent speed
 *
 * Maximum downward velocity [m/s, NED +down] the tracker's vz intent may command
 * in FOLLOW_ME. This REPLACES the gentle MPC_Z_VEL_MAX_DN clamp on the tracker
 * path: that stock limit is sized for benign person-following, and it throttles
 * an aggressive strike/merge dive (the companion can ask for tens of m/s and was
 * being capped to ~MPC_Z_VEL_MAX_DN). The vertical velocity RAMP is still bounded
 * by MPC_ACC_DOWN_MAX, so raise that too for a snappier nose-over closing rate.
 *
 * @unit m/s
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group FlightTaskFollowMe
 */
PARAM_DEFINE_FLOAT(FM_VEL_MAX_DN, 20.0f);

/**
 * FollowMe pilot roll/pitch nudge authority (operator-in-the-loop)
 *
 * Lets the pilot's roll/pitch sticks add a bounded offset ON TOP of the tracker's
 * autonomous horizontal steering in FOLLOW_ME, for fine manual aim corrections
 * (operator-guided / "OGL" style) without leaving the mode. The expo-shaped stick
 * value (gentle near center) is scaled by this gain and added to the normalized
 * tracker (pitch, roll) intent, then clamped to the [-1, 1] stick range before the
 * usual tilt rescale to MPC_TILTMAX_AIR. So this is the fraction of full manual tilt
 * authority the sticks may contribute: e.g. 0.3 = up to 30% of a full-stick tilt of
 * nudge, layered on the tracker command.
 *
 * 0 = disabled: pure tracker steering, the pilot's roll/pitch sticks are ignored
 * (the throttle stick remains blocked separately; see kBlockRcVerticalOverride).
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FlightTaskFollowMe
 */
PARAM_DEFINE_FLOAT(FM_PR_NUDGE, 0.0f);
