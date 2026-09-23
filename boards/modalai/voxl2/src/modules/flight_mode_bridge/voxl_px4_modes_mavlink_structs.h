/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, inc. All rights reserved.
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
 * voxl_px4_modes_mavlink_structs.h — vendored MAVLink payload struct layouts.
 *
 * The v2 modes protocol carries standard MAVLink offboard setpoint structs as
 * payloads (see voxl_px4_modes_pipe.h). These definitions are copied verbatim
 * (field-for-field, packed) from mavlink c_library_v2 common dialect so that
 * neither the PX4 bridge nor the example services need the full MAVLink
 * headers at build time.
 *
 * If your client already includes the real MAVLink headers
 * (<c_library_v2/common/mavlink.h>), include those FIRST — the guards below
 * then skip the vendored copies and you use the genuine article. Layouts are
 * identical either way (compile-time size asserts enforce it here).
 */
#ifndef VOXL_PX4_MODES_MAVLINK_STRUCTS_H
#define VOXL_PX4_MODES_MAVLINK_STRUCTS_H

#include <stdint.h>

#if defined(__cplusplus)
#define MODE_SIZE_ASSERT(expr, msg) static_assert(expr, msg)
#else
#define MODE_SIZE_ASSERT(expr, msg) _Static_assert(expr, msg)
#endif

/* ---- SET_POSITION_TARGET_LOCAL_NED (MAVLink msg id 84) ------------------- */
#ifndef MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84

typedef struct __attribute__((packed)) __mavlink_set_position_target_local_ned_t {
	uint32_t time_boot_ms;     /*< [ms] Timestamp (time since system boot). 0 is fine. */
	float x;                   /*< [m] X Position in NED frame */
	float y;                   /*< [m] Y Position in NED frame */
	float z;                   /*< [m] Z Position in NED frame (altitude is negative) */
	float vx;                  /*< [m/s] X velocity in NED frame */
	float vy;                  /*< [m/s] Y velocity in NED frame */
	float vz;                  /*< [m/s] Z velocity in NED frame */
	float afx;                 /*< [m/s/s] X acceleration in NED frame */
	float afy;                 /*< [m/s/s] Y acceleration in NED frame */
	float afz;                 /*< [m/s/s] Z acceleration in NED frame */
	float yaw;                 /*< [rad] yaw setpoint */
	float yaw_rate;            /*< [rad/s] yaw rate setpoint */
	uint16_t type_mask;        /*< bitmap: which dimensions to IGNORE (see below) */
	uint8_t target_system;     /*< unused over the pipe, set 0 */
	uint8_t target_component;  /*< unused over the pipe, set 0 */
	uint8_t coordinate_frame;  /*< must be MAV_FRAME_LOCAL_NED = 1 */
} mavlink_set_position_target_local_ned_t;

#endif /* MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED */

MODE_SIZE_ASSERT(sizeof(mavlink_set_position_target_local_ned_t) == 53,
		 "set_position_target_local_ned layout mismatch");

/* ---- SET_ATTITUDE_TARGET (MAVLink msg id 82) ----------------------------- */
#ifndef MAVLINK_MSG_ID_SET_ATTITUDE_TARGET
#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET 82

typedef struct __attribute__((packed)) __mavlink_set_attitude_target_t {
	uint32_t time_boot_ms;     /*< [ms] Timestamp (time since system boot). 0 is fine. */
	float q[4];                /*< attitude quaternion (w,x,y,z), NED -> body FRD */
	float body_roll_rate;      /*< [rad/s] */
	float body_pitch_rate;     /*< [rad/s] */
	float body_yaw_rate;       /*< [rad/s] */
	float thrust;              /*< collective thrust 0..1 (-1..1 if reversible) */
	uint8_t target_system;     /*< unused over the pipe, set 0 */
	uint8_t target_component;  /*< unused over the pipe, set 0 */
	uint8_t type_mask;         /*< bitmap: which dimensions to IGNORE (see below) */
	float thrust_body[3];      /*< 3D body-frame thrust, used if THRUST_BODY_SET */
} mavlink_set_attitude_target_t;

#endif /* MAVLINK_MSG_ID_SET_ATTITUDE_TARGET */

MODE_SIZE_ASSERT(sizeof(mavlink_set_attitude_target_t) == 51,
		 "set_attitude_target layout mismatch");

/*
 * type_mask bit values, prefixed to avoid clashing with the real MAVLink
 * enums (same numeric values; use either spelling).
 */
/* SET_POSITION_TARGET_LOCAL_NED.type_mask (POSITION_TARGET_TYPEMASK_*) */
#define MODE_POS_MASK_X_IGNORE        (1u << 0)
#define MODE_POS_MASK_Y_IGNORE        (1u << 1)
#define MODE_POS_MASK_Z_IGNORE        (1u << 2)
#define MODE_POS_MASK_VX_IGNORE       (1u << 3)
#define MODE_POS_MASK_VY_IGNORE       (1u << 4)
#define MODE_POS_MASK_VZ_IGNORE       (1u << 5)
#define MODE_POS_MASK_AX_IGNORE       (1u << 6)
#define MODE_POS_MASK_AY_IGNORE       (1u << 7)
#define MODE_POS_MASK_AZ_IGNORE       (1u << 8)
#define MODE_POS_MASK_FORCE_SET       (1u << 9)   /* unsupported: rejected */
#define MODE_POS_MASK_YAW_IGNORE      (1u << 10)
#define MODE_POS_MASK_YAW_RATE_IGNORE (1u << 11)

/* SET_ATTITUDE_TARGET.type_mask (ATTITUDE_TARGET_TYPEMASK_*) */
#define MODE_ATT_MASK_BODY_ROLL_RATE_IGNORE  (1u << 0)
#define MODE_ATT_MASK_BODY_PITCH_RATE_IGNORE (1u << 1)
#define MODE_ATT_MASK_BODY_YAW_RATE_IGNORE   (1u << 2)
#define MODE_ATT_MASK_THRUST_BODY_SET        (1u << 5)
#define MODE_ATT_MASK_THROTTLE_IGNORE        (1u << 6)
#define MODE_ATT_MASK_ATTITUDE_IGNORE        (1u << 7)

#define MODE_MAV_FRAME_LOCAL_NED 1

#undef MODE_SIZE_ASSERT

#endif /* VOXL_PX4_MODES_MAVLINK_STRUCTS_H */
