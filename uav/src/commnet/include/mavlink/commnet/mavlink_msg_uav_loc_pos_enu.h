#pragma once
// MESSAGE UAV_LOC_POS_ENU PACKING

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU 6001


typedef struct __mavlink_uav_loc_pos_enu_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
 float yaw; /*<  yaw(rad).*/
 uint8_t flag; /*<  参数标志*/
} mavlink_uav_loc_pos_enu_t;

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN 17
#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN 17
#define MAVLINK_MSG_ID_6001_LEN 17
#define MAVLINK_MSG_ID_6001_MIN_LEN 17

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC 44
#define MAVLINK_MSG_ID_6001_CRC 44



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_LOC_POS_ENU { \
    6001, \
    "UAV_LOC_POS_ENU", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_loc_pos_enu_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_loc_pos_enu_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_loc_pos_enu_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_loc_pos_enu_t, yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uav_loc_pos_enu_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_LOC_POS_ENU { \
    "UAV_LOC_POS_ENU", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_loc_pos_enu_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_loc_pos_enu_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_loc_pos_enu_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_loc_pos_enu_t, yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uav_loc_pos_enu_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_loc_pos_enu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @param flag  参数标志
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN);
#else
    mavlink_uav_loc_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_LOC_POS_ENU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
}

/**
 * @brief Pack a uav_loc_pos_enu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @param flag  参数标志
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float yaw,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN);
#else
    mavlink_uav_loc_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_LOC_POS_ENU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
}

/**
 * @brief Encode a uav_loc_pos_enu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_loc_pos_enu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_loc_pos_enu_t* uav_loc_pos_enu)
{
    return mavlink_msg_uav_loc_pos_enu_pack(system_id, component_id, msg, uav_loc_pos_enu->x, uav_loc_pos_enu->y, uav_loc_pos_enu->z, uav_loc_pos_enu->yaw, uav_loc_pos_enu->flag);
}

/**
 * @brief Encode a uav_loc_pos_enu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_loc_pos_enu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_loc_pos_enu_t* uav_loc_pos_enu)
{
    return mavlink_msg_uav_loc_pos_enu_pack_chan(system_id, component_id, chan, msg, uav_loc_pos_enu->x, uav_loc_pos_enu->y, uav_loc_pos_enu->z, uav_loc_pos_enu->yaw, uav_loc_pos_enu->flag);
}

/**
 * @brief Send a uav_loc_pos_enu message
 * @param chan MAVLink channel to send the message
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @param flag  参数标志
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_loc_pos_enu_send(mavlink_channel_t chan, float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU, buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
#else
    mavlink_uav_loc_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU, (const char *)&packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
#endif
}

/**
 * @brief Send a uav_loc_pos_enu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_loc_pos_enu_send_struct(mavlink_channel_t chan, const mavlink_uav_loc_pos_enu_t* uav_loc_pos_enu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_loc_pos_enu_send(chan, uav_loc_pos_enu->x, uav_loc_pos_enu->y, uav_loc_pos_enu->z, uav_loc_pos_enu->yaw, uav_loc_pos_enu->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU, (const char *)uav_loc_pos_enu, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_loc_pos_enu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU, buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
#else
    mavlink_uav_loc_pos_enu_t *packet = (mavlink_uav_loc_pos_enu_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU, (const char *)packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_LOC_POS_ENU UNPACKING


/**
 * @brief Get field x from uav_loc_pos_enu message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from uav_loc_pos_enu message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from uav_loc_pos_enu message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from uav_loc_pos_enu message
 *
 * @return  yaw(rad).
 */
static inline float mavlink_msg_uav_loc_pos_enu_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field flag from uav_loc_pos_enu message
 *
 * @return  参数标志
 */
static inline uint8_t mavlink_msg_uav_loc_pos_enu_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a uav_loc_pos_enu message into a struct
 *
 * @param msg The message to decode
 * @param uav_loc_pos_enu C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_loc_pos_enu_decode(const mavlink_message_t* msg, mavlink_uav_loc_pos_enu_t* uav_loc_pos_enu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_loc_pos_enu->x = mavlink_msg_uav_loc_pos_enu_get_x(msg);
    uav_loc_pos_enu->y = mavlink_msg_uav_loc_pos_enu_get_y(msg);
    uav_loc_pos_enu->z = mavlink_msg_uav_loc_pos_enu_get_z(msg);
    uav_loc_pos_enu->yaw = mavlink_msg_uav_loc_pos_enu_get_yaw(msg);
    uav_loc_pos_enu->flag = mavlink_msg_uav_loc_pos_enu_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN? msg->len : MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN;
        memset(uav_loc_pos_enu, 0, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_LEN);
    memcpy(uav_loc_pos_enu, _MAV_PAYLOAD(msg), len);
#endif
}
