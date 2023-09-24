#pragma once
// MESSAGE MISSION_CTRL_SET_POS PACKING

#define MAVLINK_MSG_ID_MISSION_CTRL_SET_POS 7701


typedef struct __mavlink_mission_ctrl_set_pos_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
 float yaw; /*<  yaw(rad).*/
 uint8_t flag; /*<  flag.*/
} mavlink_mission_ctrl_set_pos_t;

#define MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN 17
#define MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN 17
#define MAVLINK_MSG_ID_7701_LEN 17
#define MAVLINK_MSG_ID_7701_MIN_LEN 17

#define MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC 54
#define MAVLINK_MSG_ID_7701_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_CTRL_SET_POS { \
    7701, \
    "MISSION_CTRL_SET_POS", \
    5, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mission_ctrl_set_pos_t, flag) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mission_ctrl_set_pos_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mission_ctrl_set_pos_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_ctrl_set_pos_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_ctrl_set_pos_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_CTRL_SET_POS { \
    "MISSION_CTRL_SET_POS", \
    5, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mission_ctrl_set_pos_t, flag) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mission_ctrl_set_pos_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mission_ctrl_set_pos_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_ctrl_set_pos_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_ctrl_set_pos_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_ctrl_set_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flag  flag.
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_ctrl_set_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t flag, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN);
#else
    mavlink_mission_ctrl_set_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CTRL_SET_POS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
}

/**
 * @brief Pack a mission_ctrl_set_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flag  flag.
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_ctrl_set_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t flag,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN);
#else
    mavlink_mission_ctrl_set_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CTRL_SET_POS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
}

/**
 * @brief Encode a mission_ctrl_set_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_ctrl_set_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_ctrl_set_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_ctrl_set_pos_t* mission_ctrl_set_pos)
{
    return mavlink_msg_mission_ctrl_set_pos_pack(system_id, component_id, msg, mission_ctrl_set_pos->flag, mission_ctrl_set_pos->x, mission_ctrl_set_pos->y, mission_ctrl_set_pos->z, mission_ctrl_set_pos->yaw);
}

/**
 * @brief Encode a mission_ctrl_set_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_ctrl_set_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_ctrl_set_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_ctrl_set_pos_t* mission_ctrl_set_pos)
{
    return mavlink_msg_mission_ctrl_set_pos_pack_chan(system_id, component_id, chan, msg, mission_ctrl_set_pos->flag, mission_ctrl_set_pos->x, mission_ctrl_set_pos->y, mission_ctrl_set_pos->z, mission_ctrl_set_pos->yaw);
}

/**
 * @brief Send a mission_ctrl_set_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param flag  flag.
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_ctrl_set_pos_send(mavlink_channel_t chan, uint8_t flag, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS, buf, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
#else
    mavlink_mission_ctrl_set_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
#endif
}

/**
 * @brief Send a mission_ctrl_set_pos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_ctrl_set_pos_send_struct(mavlink_channel_t chan, const mavlink_mission_ctrl_set_pos_t* mission_ctrl_set_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_ctrl_set_pos_send(chan, mission_ctrl_set_pos->flag, mission_ctrl_set_pos->x, mission_ctrl_set_pos->y, mission_ctrl_set_pos->z, mission_ctrl_set_pos->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS, (const char *)mission_ctrl_set_pos, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_ctrl_set_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flag, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS, buf, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
#else
    mavlink_mission_ctrl_set_pos_t *packet = (mavlink_mission_ctrl_set_pos_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS, (const char *)packet, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_MIN_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_CTRL_SET_POS UNPACKING


/**
 * @brief Get field flag from mission_ctrl_set_pos message
 *
 * @return  flag.
 */
static inline uint8_t mavlink_msg_mission_ctrl_set_pos_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field x from mission_ctrl_set_pos message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_mission_ctrl_set_pos_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from mission_ctrl_set_pos message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_mission_ctrl_set_pos_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from mission_ctrl_set_pos message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_mission_ctrl_set_pos_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from mission_ctrl_set_pos message
 *
 * @return  yaw(rad).
 */
static inline float mavlink_msg_mission_ctrl_set_pos_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a mission_ctrl_set_pos message into a struct
 *
 * @param msg The message to decode
 * @param mission_ctrl_set_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_ctrl_set_pos_decode(const mavlink_message_t* msg, mavlink_mission_ctrl_set_pos_t* mission_ctrl_set_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_ctrl_set_pos->x = mavlink_msg_mission_ctrl_set_pos_get_x(msg);
    mission_ctrl_set_pos->y = mavlink_msg_mission_ctrl_set_pos_get_y(msg);
    mission_ctrl_set_pos->z = mavlink_msg_mission_ctrl_set_pos_get_z(msg);
    mission_ctrl_set_pos->yaw = mavlink_msg_mission_ctrl_set_pos_get_yaw(msg);
    mission_ctrl_set_pos->flag = mavlink_msg_mission_ctrl_set_pos_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN? msg->len : MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN;
        memset(mission_ctrl_set_pos, 0, MAVLINK_MSG_ID_MISSION_CTRL_SET_POS_LEN);
    memcpy(mission_ctrl_set_pos, _MAV_PAYLOAD(msg), len);
#endif
}
