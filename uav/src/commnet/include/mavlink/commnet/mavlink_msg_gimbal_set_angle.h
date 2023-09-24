#pragma once
// MESSAGE GIMBAL_SET_ANGLE PACKING

#define MAVLINK_MSG_ID_GIMBAL_SET_ANGLE 6037


typedef struct __mavlink_gimbal_set_angle_t {
 float desire_yaw; /*<   期望设置的yaw角度,float,一位小数,度. 示例 2.2Deg.*/
 float desire_pitch; /*<   期望设置的pitch角度,float,一位小数,度. 示例 2.2Deg.*/
} mavlink_gimbal_set_angle_t;

#define MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN 8
#define MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN 8
#define MAVLINK_MSG_ID_6037_LEN 8
#define MAVLINK_MSG_ID_6037_MIN_LEN 8

#define MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC 148
#define MAVLINK_MSG_ID_6037_CRC 148



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_SET_ANGLE { \
    6037, \
    "GIMBAL_SET_ANGLE", \
    2, \
    {  { "desire_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_set_angle_t, desire_yaw) }, \
         { "desire_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_set_angle_t, desire_pitch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_SET_ANGLE { \
    "GIMBAL_SET_ANGLE", \
    2, \
    {  { "desire_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_set_angle_t, desire_yaw) }, \
         { "desire_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_set_angle_t, desire_pitch) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_set_angle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param desire_yaw   期望设置的yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param desire_pitch   期望设置的pitch角度,float,一位小数,度. 示例 2.2Deg.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_set_angle_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float desire_yaw, float desire_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN];
    _mav_put_float(buf, 0, desire_yaw);
    _mav_put_float(buf, 4, desire_pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN);
#else
    mavlink_gimbal_set_angle_t packet;
    packet.desire_yaw = desire_yaw;
    packet.desire_pitch = desire_pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_SET_ANGLE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
}

/**
 * @brief Pack a gimbal_set_angle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param desire_yaw   期望设置的yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param desire_pitch   期望设置的pitch角度,float,一位小数,度. 示例 2.2Deg.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_set_angle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float desire_yaw,float desire_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN];
    _mav_put_float(buf, 0, desire_yaw);
    _mav_put_float(buf, 4, desire_pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN);
#else
    mavlink_gimbal_set_angle_t packet;
    packet.desire_yaw = desire_yaw;
    packet.desire_pitch = desire_pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_SET_ANGLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
}

/**
 * @brief Encode a gimbal_set_angle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_set_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_set_angle_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_set_angle_t* gimbal_set_angle)
{
    return mavlink_msg_gimbal_set_angle_pack(system_id, component_id, msg, gimbal_set_angle->desire_yaw, gimbal_set_angle->desire_pitch);
}

/**
 * @brief Encode a gimbal_set_angle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_set_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_set_angle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_set_angle_t* gimbal_set_angle)
{
    return mavlink_msg_gimbal_set_angle_pack_chan(system_id, component_id, chan, msg, gimbal_set_angle->desire_yaw, gimbal_set_angle->desire_pitch);
}

/**
 * @brief Send a gimbal_set_angle message
 * @param chan MAVLink channel to send the message
 *
 * @param desire_yaw   期望设置的yaw角度,float,一位小数,度. 示例 2.2Deg.
 * @param desire_pitch   期望设置的pitch角度,float,一位小数,度. 示例 2.2Deg.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_set_angle_send(mavlink_channel_t chan, float desire_yaw, float desire_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN];
    _mav_put_float(buf, 0, desire_yaw);
    _mav_put_float(buf, 4, desire_pitch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE, buf, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
#else
    mavlink_gimbal_set_angle_t packet;
    packet.desire_yaw = desire_yaw;
    packet.desire_pitch = desire_pitch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
#endif
}

/**
 * @brief Send a gimbal_set_angle message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_set_angle_send_struct(mavlink_channel_t chan, const mavlink_gimbal_set_angle_t* gimbal_set_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_set_angle_send(chan, gimbal_set_angle->desire_yaw, gimbal_set_angle->desire_pitch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE, (const char *)gimbal_set_angle, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_set_angle_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float desire_yaw, float desire_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, desire_yaw);
    _mav_put_float(buf, 4, desire_pitch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE, buf, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
#else
    mavlink_gimbal_set_angle_t *packet = (mavlink_gimbal_set_angle_t *)msgbuf;
    packet->desire_yaw = desire_yaw;
    packet->desire_pitch = desire_pitch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_SET_ANGLE UNPACKING


/**
 * @brief Get field desire_yaw from gimbal_set_angle message
 *
 * @return   期望设置的yaw角度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_set_angle_get_desire_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field desire_pitch from gimbal_set_angle message
 *
 * @return   期望设置的pitch角度,float,一位小数,度. 示例 2.2Deg.
 */
static inline float mavlink_msg_gimbal_set_angle_get_desire_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a gimbal_set_angle message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_set_angle C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_set_angle_decode(const mavlink_message_t* msg, mavlink_gimbal_set_angle_t* gimbal_set_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_set_angle->desire_yaw = mavlink_msg_gimbal_set_angle_get_desire_yaw(msg);
    gimbal_set_angle->desire_pitch = mavlink_msg_gimbal_set_angle_get_desire_pitch(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN;
        memset(gimbal_set_angle, 0, MAVLINK_MSG_ID_GIMBAL_SET_ANGLE_LEN);
    memcpy(gimbal_set_angle, _MAV_PAYLOAD(msg), len);
#endif
}
