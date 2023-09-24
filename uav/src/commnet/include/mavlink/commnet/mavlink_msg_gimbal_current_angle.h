#pragma once
// MESSAGE GIMBAL_CURRENT_ANGLE PACKING

#define MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE 6036


typedef struct __mavlink_gimbal_current_angle_t {
 float yaw; /*<   yaw角度,float,一位小数,度. 示例 2.2Deg.*/
 float pitch; /*<   pitch角度,float,一位小数,度. 示例 2.2Deg.*/
 float roll; /*<   roll角度,float,一位小数,度. 示例 2.2Deg.*/
 float yawvel; /*<   yaw角速度,float,一位小数,度. 示例 2.2Deg.*/
 float pitchvel; /*<   pitch角速度,float,一位小数,度. 示例 2.2Deg.*/
 float rollvel; /*<   roll角速度,float,一位小数,度. 示例 2.2Deg.*/
} mavlink_gimbal_current_angle_t;

#define MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN 24
#define MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN 24
#define MAVLINK_MSG_ID_6036_LEN 24
#define MAVLINK_MSG_ID_6036_MIN_LEN 24

#define MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC 47
#define MAVLINK_MSG_ID_6036_CRC 47



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_CURRENT_ANGLE { \
    6036, \
    "GIMBAL_CURRENT_ANGLE", \
    6, \
    {  { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_current_angle_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_current_angle_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_current_angle_t, roll) }, \
         { "yawvel", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_current_angle_t, yawvel) }, \
         { "pitchvel", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_current_angle_t, pitchvel) }, \
         { "rollvel", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_current_angle_t, rollvel) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_CURRENT_ANGLE { \
    "GIMBAL_CURRENT_ANGLE", \
    6, \
    {  { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_current_angle_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_current_angle_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_current_angle_t, roll) }, \
         { "yawvel", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_current_angle_t, yawvel) }, \
         { "pitchvel", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_current_angle_t, pitchvel) }, \
         { "rollvel", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_current_angle_t, rollvel) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_current_angle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yaw   yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param pitch   pitch角度,float,一位小数,度. 示例 2.2Deg.
 * @param roll   roll角度,float,一位小数,度. 示例 2.2Deg.
 * @param yawvel   yaw角速度,float,一位小数,度. 示例 2.2Deg.
 * @param pitchvel   pitch角速度,float,一位小数,度. 示例 2.2Deg.
 * @param rollvel   roll角速度,float,一位小数,度. 示例 2.2Deg.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_current_angle_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float yaw, float pitch, float roll, float yawvel, float pitchvel, float rollvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, yawvel);
    _mav_put_float(buf, 16, pitchvel);
    _mav_put_float(buf, 20, rollvel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN);
#else
    mavlink_gimbal_current_angle_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yawvel = yawvel;
    packet.pitchvel = pitchvel;
    packet.rollvel = rollvel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
}

/**
 * @brief Pack a gimbal_current_angle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yaw   yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param pitch   pitch角度,float,一位小数,度. 示例 2.2Deg.
 * @param roll   roll角度,float,一位小数,度. 示例 2.2Deg.
 * @param yawvel   yaw角速度,float,一位小数,度. 示例 2.2Deg.
 * @param pitchvel   pitch角速度,float,一位小数,度. 示例 2.2Deg.
 * @param rollvel   roll角速度,float,一位小数,度. 示例 2.2Deg.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_current_angle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float yaw,float pitch,float roll,float yawvel,float pitchvel,float rollvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, yawvel);
    _mav_put_float(buf, 16, pitchvel);
    _mav_put_float(buf, 20, rollvel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN);
#else
    mavlink_gimbal_current_angle_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yawvel = yawvel;
    packet.pitchvel = pitchvel;
    packet.rollvel = rollvel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
}

/**
 * @brief Encode a gimbal_current_angle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_current_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_current_angle_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_current_angle_t* gimbal_current_angle)
{
    return mavlink_msg_gimbal_current_angle_pack(system_id, component_id, msg, gimbal_current_angle->yaw, gimbal_current_angle->pitch, gimbal_current_angle->roll, gimbal_current_angle->yawvel, gimbal_current_angle->pitchvel, gimbal_current_angle->rollvel);
}

/**
 * @brief Encode a gimbal_current_angle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_current_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_current_angle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_current_angle_t* gimbal_current_angle)
{
    return mavlink_msg_gimbal_current_angle_pack_chan(system_id, component_id, chan, msg, gimbal_current_angle->yaw, gimbal_current_angle->pitch, gimbal_current_angle->roll, gimbal_current_angle->yawvel, gimbal_current_angle->pitchvel, gimbal_current_angle->rollvel);
}

/**
 * @brief Send a gimbal_current_angle message
 * @param chan MAVLink channel to send the message
 *
 * @param yaw   yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param pitch   pitch角度,float,一位小数,度. 示例 2.2Deg.
 * @param roll   roll角度,float,一位小数,度. 示例 2.2Deg.
 * @param yawvel   yaw角速度,float,一位小数,度. 示例 2.2Deg.
 * @param pitchvel   pitch角速度,float,一位小数,度. 示例 2.2Deg.
 * @param rollvel   roll角速度,float,一位小数,度. 示例 2.2Deg.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_current_angle_send(mavlink_channel_t chan, float yaw, float pitch, float roll, float yawvel, float pitchvel, float rollvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, yawvel);
    _mav_put_float(buf, 16, pitchvel);
    _mav_put_float(buf, 20, rollvel);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE, buf, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
#else
    mavlink_gimbal_current_angle_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yawvel = yawvel;
    packet.pitchvel = pitchvel;
    packet.rollvel = rollvel;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
#endif
}

/**
 * @brief Send a gimbal_current_angle message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_current_angle_send_struct(mavlink_channel_t chan, const mavlink_gimbal_current_angle_t* gimbal_current_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_current_angle_send(chan, gimbal_current_angle->yaw, gimbal_current_angle->pitch, gimbal_current_angle->roll, gimbal_current_angle->yawvel, gimbal_current_angle->pitchvel, gimbal_current_angle->rollvel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE, (const char *)gimbal_current_angle, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_current_angle_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float yaw, float pitch, float roll, float yawvel, float pitchvel, float rollvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, yawvel);
    _mav_put_float(buf, 16, pitchvel);
    _mav_put_float(buf, 20, rollvel);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE, buf, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
#else
    mavlink_gimbal_current_angle_t *packet = (mavlink_gimbal_current_angle_t *)msgbuf;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->yawvel = yawvel;
    packet->pitchvel = pitchvel;
    packet->rollvel = rollvel;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_CURRENT_ANGLE UNPACKING


/**
 * @brief Get field yaw from gimbal_current_angle message
 *
 * @return   yaw角度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from gimbal_current_angle message
 *
 * @return   pitch角度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll from gimbal_current_angle message
 *
 * @return   roll角度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yawvel from gimbal_current_angle message
 *
 * @return   yaw角速度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_yawvel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitchvel from gimbal_current_angle message
 *
 * @return   pitch角速度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_pitchvel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollvel from gimbal_current_angle message
 *
 * @return   roll角速度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_current_angle_get_rollvel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a gimbal_current_angle message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_current_angle C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_current_angle_decode(const mavlink_message_t* msg, mavlink_gimbal_current_angle_t* gimbal_current_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_current_angle->yaw = mavlink_msg_gimbal_current_angle_get_yaw(msg);
    gimbal_current_angle->pitch = mavlink_msg_gimbal_current_angle_get_pitch(msg);
    gimbal_current_angle->roll = mavlink_msg_gimbal_current_angle_get_roll(msg);
    gimbal_current_angle->yawvel = mavlink_msg_gimbal_current_angle_get_yawvel(msg);
    gimbal_current_angle->pitchvel = mavlink_msg_gimbal_current_angle_get_pitchvel(msg);
    gimbal_current_angle->rollvel = mavlink_msg_gimbal_current_angle_get_rollvel(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN;
        memset(gimbal_current_angle, 0, MAVLINK_MSG_ID_GIMBAL_CURRENT_ANGLE_LEN);
    memcpy(gimbal_current_angle, _MAV_PAYLOAD(msg), len);
#endif
}
