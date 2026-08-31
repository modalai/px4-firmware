/**
 * voxl_px4_modes_pipe.h — wire protocol for VOXL external flight modes.
 *
 * Contract between apps-side mode clients and the flight_mode_bridge module
 * inside voxl-px4. The ORIGINAL of this file lives in the apps-side example
 * repo (currently voxl-figure-eight); the PX4 board tree carries a synced
 * copy. Make changes in the original and copy it over — keep the two
 * identical (compare VOXL_PX4_MODES_PROTOCOL_VERSION on both sides).
 *
 * Transport: one MPA server pipe served by the bridge ("px4_modes").
 *   client -> bridge : control messages on the control pipe
 *   bridge -> client : messages written to the data pipe (broadcast to all)
 *
 * Design:
 *  - Setpoint payloads are standard MAVLink offboard structs
 *    (SET_POSITION_TARGET_LOCAL_NED / SET_ATTITUDE_TARGET), carried verbatim
 *    after the request_id. Clients that already speak MAVLink offboard keep
 *    their setpoint-building code unchanged. Include the real MAVLink headers
 *    (<c_library_v2/common/mavlink.h>, voxl-mavlink package) for the structs.
 *  - REGISTER_REQ declares a setpoint type (trajectory or attitude). The
 *    bridge forwards it to PX4 (SetpointConfig), which derives the control
 *    flags for the mode and arms a 500 ms setpoint-loss failsafe.
 *  - MODE_MSG_VEHICLE_STATE: bridge broadcasts vehicle state at ~50 Hz while
 *    any client is registered (fused local position/velocity, attitude,
 *    pilot sticks) so controller-style modes need no other data source.
 *
 * Lifecycle: register (retry at 1 Hz until acked) -> stream
 * HEALTH at 1 Hz -> on MODE_STATE(active) stream setpoints at your rate ->
 * on MODE_STATE(inactive) stop -> UNREGISTER on clean exit. All messages are
 * prefixed by the common header below; every payload begins with the client's
 * random 64-bit request_id (the control pipe carries no sender identity).
 */
#ifndef VOXL_PX4_MODES_PIPE_H
#define VOXL_PX4_MODES_PIPE_H

#include <stdint.h>

#define VOXL_PX4_MODES_PIPE_NAME        "px4_modes"
#define VOXL_PX4_MODES_MAGIC            0x4D783450  // "Px4M"
#define VOXL_PX4_MODES_PROTOCOL_VERSION 1
#define VOXL_PX4_MODES_NAME_LEN         25          // matches uORB char[25] name

typedef enum mode_msg_type_t {
	MODE_MSG_REGISTER_REQ   = 1,  // client -> bridge
	MODE_MSG_REGISTER_REPLY = 2,  // bridge -> client
	MODE_MSG_HEALTH         = 3,  // client -> bridge (>=1 Hz while idle; cached by bridge)
	MODE_MSG_MODE_STATE     = 4,  // bridge -> client (activation/deactivation + snapshot)
	MODE_MSG_TRAJ_SETPOINT  = 5,  // client -> bridge: request_id + mavlink_set_position_target_local_ned_t
	MODE_MSG_UNREGISTER     = 6,  // client -> bridge
	MODE_MSG_ATT_SETPOINT   = 7,  // client -> bridge: request_id + mavlink_set_attitude_target_t
	MODE_MSG_VEHICLE_STATE  = 8,  // bridge -> client (broadcast, ~50 Hz)
} mode_msg_type_t;

// setpoint type declared at registration. Values match PX4's SetpointConfig
// TYPE_* so the bridge forwards them verbatim.
#define MODE_SETPOINT_TRAJECTORY 4  // stream MODE_MSG_TRAJ_SETPOINT; PX4 runs position control
#define MODE_SETPOINT_ATTITUDE   6  // stream MODE_MSG_ATT_SETPOINT; you run position control

// common header prefixed to every message
typedef struct __attribute__((packed)) mode_msg_header_t {
	uint32_t magic;          // VOXL_PX4_MODES_MAGIC
	uint8_t  version;        // VOXL_PX4_MODES_PROTOCOL_VERSION
	uint8_t  type;           // mode_msg_type_t
	uint16_t payload_bytes;  // size of payload that follows this header
} mode_msg_header_t;

// ---- client -> bridge ------------------------------------------------------

typedef struct __attribute__((packed)) mode_register_req_t {
	uint64_t request_id;                     // random; echoed in reply
	char     name[VOXL_PX4_MODES_NAME_LEN];  // mode name shown in GCS
	uint8_t  register_arming_check;          // must be 1 when registering a mode
	uint8_t  register_mode;
	uint8_t  setpoint_type;                  // MODE_SETPOINT_*
} mode_register_req_t;

// requirement flag bits for mode_health_t.mode_requirements
// (mirror ArmingCheckReply mode_req_* booleans)
#define MODE_REQ_ANGULAR_VELOCITY  (1u<<0)
#define MODE_REQ_ATTITUDE          (1u<<1)
#define MODE_REQ_LOCAL_ALT         (1u<<2)
#define MODE_REQ_LOCAL_POSITION    (1u<<3)
#define MODE_REQ_GLOBAL_POSITION   (1u<<5)
#define MODE_REQ_MANUAL_CONTROL    (1u<<11)

typedef struct __attribute__((packed)) mode_health_t {
	uint64_t request_id;         // which registration this refers to
	uint8_t  can_arm_and_run;    // 1 = healthy
	uint32_t mode_requirements;  // MODE_REQ_* bits
} mode_health_t;

/*
 * MODE_MSG_TRAJ_SETPOINT payload: uint64_t request_id immediately followed by
 * a packed mavlink_set_position_target_local_ned_t (coordinate_frame must be
 * MAV_FRAME_LOCAL_NED; type_mask semantics identical to MAVLink offboard;
 * FORCE_SET unsupported).
 *
 * MODE_MSG_ATT_SETPOINT payload: uint64_t request_id immediately followed by
 * a packed mavlink_set_attitude_target_t (attitude + thrust required; body
 * roll/pitch rate fields ignored — rates rung not yet supported).
 */

typedef struct __attribute__((packed)) mode_unregister_t {
	uint64_t request_id;
} mode_unregister_t;

// ---- bridge -> client ------------------------------------------------------

typedef struct __attribute__((packed)) mode_register_reply_t {
	uint64_t request_id;         // echoed from request
	uint8_t  success;
	int8_t   mode_id;            // assigned nav_state (23..30), -1 invalid
	int8_t   arming_check_id;    // -1 invalid
} mode_register_reply_t;

typedef struct __attribute__((packed)) mode_state_t {
	uint64_t request_id;
	uint8_t  active;             // 1 = your mode is now flying, 0 = deactivated
	uint8_t  armed;              // vehicle armed state at this moment
	float    position[3];        // vehicle local position NED at (de)activation [m]
	float    yaw;                // vehicle yaw at (de)activation [rad]
} mode_state_t;

// broadcast to all clients at ~50 Hz while any client is registered.
// PX4's fused estimate — same frame the controllers use, so setpoints built
// from this state need no frame alignment.
typedef struct __attribute__((packed)) mode_vehicle_state_t {
	uint64_t timestamp_us;       // PX4 monotonic time of this sample
	float    position[3];        // m, local NED
	float    velocity[3];        // m/s, local NED
	float    q[4];               // attitude quaternion body FRD -> NED (w,x,y,z)
	float    heading;            // rad, yaw extracted for convenience
	float    stick_roll;         // pilot sticks, [-1, 1] each
	float    stick_pitch;
	float    stick_yaw;
	float    stick_throttle;
	float    aux[6];             // aux channels (levers/switches), normalized
	uint8_t  manual_valid;       // 1 = stick fields are current
	uint8_t  armed;
	uint8_t  nav_state;          // current PX4 nav_state (23..30 = EXTERNAL1..8)
} mode_vehicle_state_t;

#endif // VOXL_PX4_MODES_PIPE_H
