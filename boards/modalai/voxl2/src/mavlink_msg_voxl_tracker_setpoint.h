#pragma once
// MESSAGE VOXL_TRACKER_SETPOINT PACKING
//
// VENDORED ModalAI custom MAVLink message (kept in lock-step with the voxl-vision-hub
// copy: include/mavlink_msg_voxl_tracker_setpoint.h). It is deliberately NOT added to any
// MAVLink dialect XML, so it does not appear in the generated message-CRC table. PX4's
// parser therefore looks up msgid 12200, finds no entry, and validates the frame with
// crc_extra = 0 (see mavlink_get_msg_entry / mavlink_frame_char_buffer). The sender
// (voxl-vision-hub) finalizes with crc_extra = 0 as well, so the frame passes CRC at every
// unaware hop (voxl-mavlink-server, PX4) and is delivered to mavlink_receiver, which then
// decodes it using this header. Keep CRC_EXTRA = 0 in BOTH copies. If this message is ever
// registered in a dialect, switch both copies to the generated CRC_EXTRA (186) instead.

#define MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT 12200


typedef struct __mavlink_voxl_tracker_setpoint_t {
 uint64_t time_usec; /*< [us] Timestamp of the tracker observation this intent is derived from.*/
 float roll_body; /*<  Right(+)/left(-) steering intent, normalized [-1,1] (interpreted like a roll stick).*/
 float pitch_body; /*<  Forward(+)/back(-) steering intent, normalized [-1,1] (interpreted like a pitch stick).*/
 float yaw_rate; /*< [rad/s] Desired yaw rate (+ = yaw right). ~0 locks heading.*/
 float vz; /*< [m/s] Desired vertical velocity, NED frame (+ = down). ~0 => PX4 holds altitude.*/
 float error_x; /*<  Horizontal tracking error, normalized image frame (logging / feed-forward only).*/
 float error_y; /*<  Vertical tracking error, normalized image frame (logging / feed-forward only).*/
 float error_x_rate; /*<  Time derivative of error_x.*/
 float error_y_rate; /*<  Time derivative of error_y.*/
 float confidence; /*<  Tracker confidence, 0..1.*/
 uint8_t valid; /*<  1 if roll_body/pitch_body/yaw_rate/vz are usable this cycle, otherwise 0.*/
} mavlink_voxl_tracker_setpoint_t;

#define MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN 45
#define MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_MIN_LEN 45
#define MAVLINK_MSG_ID_12200_LEN 45
#define MAVLINK_MSG_ID_12200_MIN_LEN 45

// Private message (not in any dialect) => crc_extra 0 so unaware parsers accept it. See note above.
#define MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_CRC 0
#define MAVLINK_MSG_ID_12200_CRC 0



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VOXL_TRACKER_SETPOINT { \
    12200, \
    "VOXL_TRACKER_SETPOINT", \
    11, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_voxl_tracker_setpoint_t, time_usec) }, \
         { "roll_body", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_voxl_tracker_setpoint_t, roll_body) }, \
         { "pitch_body", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_voxl_tracker_setpoint_t, pitch_body) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_voxl_tracker_setpoint_t, yaw_rate) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_voxl_tracker_setpoint_t, vz) }, \
         { "error_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_voxl_tracker_setpoint_t, error_x) }, \
         { "error_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_voxl_tracker_setpoint_t, error_y) }, \
         { "error_x_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_voxl_tracker_setpoint_t, error_x_rate) }, \
         { "error_y_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_voxl_tracker_setpoint_t, error_y_rate) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_voxl_tracker_setpoint_t, confidence) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_voxl_tracker_setpoint_t, valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VOXL_TRACKER_SETPOINT { \
    "VOXL_TRACKER_SETPOINT", \
    11, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_voxl_tracker_setpoint_t, time_usec) }, \
         { "roll_body", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_voxl_tracker_setpoint_t, roll_body) }, \
         { "pitch_body", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_voxl_tracker_setpoint_t, pitch_body) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_voxl_tracker_setpoint_t, yaw_rate) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_voxl_tracker_setpoint_t, vz) }, \
         { "error_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_voxl_tracker_setpoint_t, error_x) }, \
         { "error_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_voxl_tracker_setpoint_t, error_y) }, \
         { "error_x_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_voxl_tracker_setpoint_t, error_x_rate) }, \
         { "error_y_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_voxl_tracker_setpoint_t, error_y_rate) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_voxl_tracker_setpoint_t, confidence) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_voxl_tracker_setpoint_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a voxl_tracker_setpoint message (crc_extra 0; private message)
 */
static inline uint16_t mavlink_msg_voxl_tracker_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float roll_body, float pitch_body, float yaw_rate, float vz, float error_x, float error_y, float error_x_rate, float error_y_rate, float confidence, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll_body);
    _mav_put_float(buf, 12, pitch_body);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_float(buf, 20, vz);
    _mav_put_float(buf, 24, error_x);
    _mav_put_float(buf, 28, error_y);
    _mav_put_float(buf, 32, error_x_rate);
    _mav_put_float(buf, 36, error_y_rate);
    _mav_put_float(buf, 40, confidence);
    _mav_put_uint8_t(buf, 44, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN);
#else
    mavlink_voxl_tracker_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.roll_body = roll_body;
    packet.pitch_body = pitch_body;
    packet.yaw_rate = yaw_rate;
    packet.vz = vz;
    packet.error_x = error_x;
    packet.error_y = error_y;
    packet.error_x_rate = error_x_rate;
    packet.error_y_rate = error_y_rate;
    packet.confidence = confidence;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_CRC);
}

/**
 * @brief Encode a voxl_tracker_setpoint struct
 */
static inline uint16_t mavlink_msg_voxl_tracker_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_voxl_tracker_setpoint_t* voxl_tracker_setpoint)
{
    return mavlink_msg_voxl_tracker_setpoint_pack(system_id, component_id, msg, voxl_tracker_setpoint->time_usec, voxl_tracker_setpoint->roll_body, voxl_tracker_setpoint->pitch_body, voxl_tracker_setpoint->yaw_rate, voxl_tracker_setpoint->vz, voxl_tracker_setpoint->error_x, voxl_tracker_setpoint->error_y, voxl_tracker_setpoint->error_x_rate, voxl_tracker_setpoint->error_y_rate, voxl_tracker_setpoint->confidence, voxl_tracker_setpoint->valid);
}

// MESSAGE VOXL_TRACKER_SETPOINT UNPACKING

static inline uint64_t mavlink_msg_voxl_tracker_setpoint_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_roll_body(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_pitch_body(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_error_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_error_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_error_x_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_error_y_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

static inline float mavlink_msg_voxl_tracker_setpoint_get_confidence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

static inline uint8_t mavlink_msg_voxl_tracker_setpoint_get_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Decode a voxl_tracker_setpoint message into a struct
 */
static inline void mavlink_msg_voxl_tracker_setpoint_decode(const mavlink_message_t* msg, mavlink_voxl_tracker_setpoint_t* voxl_tracker_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    voxl_tracker_setpoint->time_usec = mavlink_msg_voxl_tracker_setpoint_get_time_usec(msg);
    voxl_tracker_setpoint->roll_body = mavlink_msg_voxl_tracker_setpoint_get_roll_body(msg);
    voxl_tracker_setpoint->pitch_body = mavlink_msg_voxl_tracker_setpoint_get_pitch_body(msg);
    voxl_tracker_setpoint->yaw_rate = mavlink_msg_voxl_tracker_setpoint_get_yaw_rate(msg);
    voxl_tracker_setpoint->vz = mavlink_msg_voxl_tracker_setpoint_get_vz(msg);
    voxl_tracker_setpoint->error_x = mavlink_msg_voxl_tracker_setpoint_get_error_x(msg);
    voxl_tracker_setpoint->error_y = mavlink_msg_voxl_tracker_setpoint_get_error_y(msg);
    voxl_tracker_setpoint->error_x_rate = mavlink_msg_voxl_tracker_setpoint_get_error_x_rate(msg);
    voxl_tracker_setpoint->error_y_rate = mavlink_msg_voxl_tracker_setpoint_get_error_y_rate(msg);
    voxl_tracker_setpoint->confidence = mavlink_msg_voxl_tracker_setpoint_get_confidence(msg);
    voxl_tracker_setpoint->valid = mavlink_msg_voxl_tracker_setpoint_get_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN;
        memset(voxl_tracker_setpoint, 0, MAVLINK_MSG_ID_VOXL_TRACKER_SETPOINT_LEN);
    memcpy(voxl_tracker_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
