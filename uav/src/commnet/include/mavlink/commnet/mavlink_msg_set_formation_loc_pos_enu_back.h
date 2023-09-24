#pragma once
// MESSAGE SET_FORMATION_LOC_POS_ENU_BACK PACKING

#define MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK 8602


typedef struct __mavlink_set_formation_loc_pos_enu_back_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
 float yaw; /*<  yaw(rad).*/
 uint8_t flag; /*<  参数标志*/
} mavlink_set_formation_loc_pos_enu_back_t;

#define MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN 17
#define MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN 17
#define MAVLINK_MSG_ID_8602_LEN 17
#define MAVLINK_MSG_ID_8602_MIN_LEN 17

#define MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC 105
#define MAVLINK_MSG_ID_8602_CRC 105



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_FORMATION_LOC_POS_ENU_BACK { \
    8602, \
    "SET_FORMATION_LOC_POS_ENU_BACK", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_formation_loc_pos_enu_back_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_formation_loc_pos_enu_back_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_formation_loc_pos_enu_back_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_formation_loc_pos_enu_back_t, yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_formation_loc_pos_enu_back_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_FORMATION_LOC_POS_ENU_BACK { \
    "SET_FORMATION_LOC_POS_ENU_BACK", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_formation_loc_pos_enu_back_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_formation_loc_pos_enu_back_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_formation_loc_pos_enu_back_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_formation_loc_pos_enu_back_t, yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_formation_loc_pos_enu_back_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a set_formation_loc_pos_enu_back message
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
static inline uint16_t mavlink_msg_set_formation_loc_pos_enu_back_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN);
#else
    mavlink_set_formation_loc_pos_enu_back_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
}

/**
 * @brief Pack a set_formation_loc_pos_enu_back message on a channel
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
static inline uint16_t mavlink_msg_set_formation_loc_pos_enu_back_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float yaw,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN);
#else
    mavlink_set_formation_loc_pos_enu_back_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
}

/**
 * @brief Encode a set_formation_loc_pos_enu_back struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_formation_loc_pos_enu_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_formation_loc_pos_enu_back_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_formation_loc_pos_enu_back_t* set_formation_loc_pos_enu_back)
{
    return mavlink_msg_set_formation_loc_pos_enu_back_pack(system_id, component_id, msg, set_formation_loc_pos_enu_back->x, set_formation_loc_pos_enu_back->y, set_formation_loc_pos_enu_back->z, set_formation_loc_pos_enu_back->yaw, set_formation_loc_pos_enu_back->flag);
}

/**
 * @brief Encode a set_formation_loc_pos_enu_back struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_formation_loc_pos_enu_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_formation_loc_pos_enu_back_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_formation_loc_pos_enu_back_t* set_formation_loc_pos_enu_back)
{
    return mavlink_msg_set_formation_loc_pos_enu_back_pack_chan(system_id, component_id, chan, msg, set_formation_loc_pos_enu_back->x, set_formation_loc_pos_enu_back->y, set_formation_loc_pos_enu_back->z, set_formation_loc_pos_enu_back->yaw, set_formation_loc_pos_enu_back->flag);
}

/**
 * @brief Send a set_formation_loc_pos_enu_back message
 * @param chan MAVLink channel to send the message
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @param flag  参数标志
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_formation_loc_pos_enu_back_send(mavlink_channel_t chan, float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK, buf, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
#else
    mavlink_set_formation_loc_pos_enu_back_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK, (const char *)&packet, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
#endif
}

/**
 * @brief Send a set_formation_loc_pos_enu_back message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_formation_loc_pos_enu_back_send_struct(mavlink_channel_t chan, const mavlink_set_formation_loc_pos_enu_back_t* set_formation_loc_pos_enu_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_formation_loc_pos_enu_back_send(chan, set_formation_loc_pos_enu_back->x, set_formation_loc_pos_enu_back->y, set_formation_loc_pos_enu_back->z, set_formation_loc_pos_enu_back->yaw, set_formation_loc_pos_enu_back->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK, (const char *)set_formation_loc_pos_enu_back, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_formation_loc_pos_enu_back_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint8_t(buf, 16, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK, buf, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
#else
    mavlink_set_formation_loc_pos_enu_back_t *packet = (mavlink_set_formation_loc_pos_enu_back_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK, (const char *)packet, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_MIN_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_FORMATION_LOC_POS_ENU_BACK UNPACKING


/**
 * @brief Get field x from set_formation_loc_pos_enu_back message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_set_formation_loc_pos_enu_back_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from set_formation_loc_pos_enu_back message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_set_formation_loc_pos_enu_back_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from set_formation_loc_pos_enu_back message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_set_formation_loc_pos_enu_back_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from set_formation_loc_pos_enu_back message
 *
 * @return  yaw(rad).
 */
static inline float mavlink_msg_set_formation_loc_pos_enu_back_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field flag from set_formation_loc_pos_enu_back message
 *
 * @return  参数标志
 */
static inline uint8_t mavlink_msg_set_formation_loc_pos_enu_back_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a set_formation_loc_pos_enu_back message into a struct
 *
 * @param msg The message to decode
 * @param set_formation_loc_pos_enu_back C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_formation_loc_pos_enu_back_decode(const mavlink_message_t* msg, mavlink_set_formation_loc_pos_enu_back_t* set_formation_loc_pos_enu_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_formation_loc_pos_enu_back->x = mavlink_msg_set_formation_loc_pos_enu_back_get_x(msg);
    set_formation_loc_pos_enu_back->y = mavlink_msg_set_formation_loc_pos_enu_back_get_y(msg);
    set_formation_loc_pos_enu_back->z = mavlink_msg_set_formation_loc_pos_enu_back_get_z(msg);
    set_formation_loc_pos_enu_back->yaw = mavlink_msg_set_formation_loc_pos_enu_back_get_yaw(msg);
    set_formation_loc_pos_enu_back->flag = mavlink_msg_set_formation_loc_pos_enu_back_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN? msg->len : MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN;
        memset(set_formation_loc_pos_enu_back, 0, MAVLINK_MSG_ID_SET_FORMATION_LOC_POS_ENU_BACK_LEN);
    memcpy(set_formation_loc_pos_enu_back, _MAV_PAYLOAD(msg), len);
#endif
}
