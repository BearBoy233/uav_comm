#pragma once
// MESSAGE UAV_LOC_POS_ENU_YAW PACKING

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW 6012


typedef struct __mavlink_uav_loc_pos_enu_yaw_t {
 float x; /*<   ENU x(m).*/
 float y; /*<   ENU y(m).*/
 float z; /*<   ENU z(m).*/
 float yaw; /*<   ENU yaw(rad).*/
} mavlink_uav_loc_pos_enu_yaw_t;

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN 16
#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN 16
#define MAVLINK_MSG_ID_6012_LEN 16
#define MAVLINK_MSG_ID_6012_MIN_LEN 16

#define MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC 79
#define MAVLINK_MSG_ID_6012_CRC 79



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_LOC_POS_ENU_YAW { \
    6012, \
    "UAV_LOC_POS_ENU_YAW", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_loc_pos_enu_yaw_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_loc_pos_enu_yaw_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_loc_pos_enu_yaw_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_loc_pos_enu_yaw_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_LOC_POS_ENU_YAW { \
    "UAV_LOC_POS_ENU_YAW", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_loc_pos_enu_yaw_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_loc_pos_enu_yaw_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_loc_pos_enu_yaw_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_loc_pos_enu_yaw_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_loc_pos_enu_yaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x   ENU x(m).
 * @param y   ENU y(m).
 * @param z   ENU z(m).
 * @param yaw   ENU yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_yaw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN);
#else
    mavlink_uav_loc_pos_enu_yaw_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
}

/**
 * @brief Pack a uav_loc_pos_enu_yaw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x   ENU x(m).
 * @param y   ENU y(m).
 * @param z   ENU z(m).
 * @param yaw   ENU yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_yaw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN);
#else
    mavlink_uav_loc_pos_enu_yaw_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
}

/**
 * @brief Encode a uav_loc_pos_enu_yaw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_loc_pos_enu_yaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_yaw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_loc_pos_enu_yaw_t* uav_loc_pos_enu_yaw)
{
    return mavlink_msg_uav_loc_pos_enu_yaw_pack(system_id, component_id, msg, uav_loc_pos_enu_yaw->x, uav_loc_pos_enu_yaw->y, uav_loc_pos_enu_yaw->z, uav_loc_pos_enu_yaw->yaw);
}

/**
 * @brief Encode a uav_loc_pos_enu_yaw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_loc_pos_enu_yaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_loc_pos_enu_yaw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_loc_pos_enu_yaw_t* uav_loc_pos_enu_yaw)
{
    return mavlink_msg_uav_loc_pos_enu_yaw_pack_chan(system_id, component_id, chan, msg, uav_loc_pos_enu_yaw->x, uav_loc_pos_enu_yaw->y, uav_loc_pos_enu_yaw->z, uav_loc_pos_enu_yaw->yaw);
}

/**
 * @brief Send a uav_loc_pos_enu_yaw message
 * @param chan MAVLink channel to send the message
 *
 * @param x   ENU x(m).
 * @param y   ENU y(m).
 * @param z   ENU z(m).
 * @param yaw   ENU yaw(rad).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_loc_pos_enu_yaw_send(mavlink_channel_t chan, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW, buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
#else
    mavlink_uav_loc_pos_enu_yaw_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW, (const char *)&packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
#endif
}

/**
 * @brief Send a uav_loc_pos_enu_yaw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_loc_pos_enu_yaw_send_struct(mavlink_channel_t chan, const mavlink_uav_loc_pos_enu_yaw_t* uav_loc_pos_enu_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_loc_pos_enu_yaw_send(chan, uav_loc_pos_enu_yaw->x, uav_loc_pos_enu_yaw->y, uav_loc_pos_enu_yaw->z, uav_loc_pos_enu_yaw->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW, (const char *)uav_loc_pos_enu_yaw, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_loc_pos_enu_yaw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW, buf, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
#else
    mavlink_uav_loc_pos_enu_yaw_t *packet = (mavlink_uav_loc_pos_enu_yaw_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW, (const char *)packet, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_MIN_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_LOC_POS_ENU_YAW UNPACKING


/**
 * @brief Get field x from uav_loc_pos_enu_yaw message
 *
 * @return   ENU x(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_yaw_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from uav_loc_pos_enu_yaw message
 *
 * @return   ENU y(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_yaw_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from uav_loc_pos_enu_yaw message
 *
 * @return   ENU z(m).
 */
static inline float mavlink_msg_uav_loc_pos_enu_yaw_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from uav_loc_pos_enu_yaw message
 *
 * @return   ENU yaw(rad).
 */
static inline float mavlink_msg_uav_loc_pos_enu_yaw_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a uav_loc_pos_enu_yaw message into a struct
 *
 * @param msg The message to decode
 * @param uav_loc_pos_enu_yaw C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_loc_pos_enu_yaw_decode(const mavlink_message_t* msg, mavlink_uav_loc_pos_enu_yaw_t* uav_loc_pos_enu_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_loc_pos_enu_yaw->x = mavlink_msg_uav_loc_pos_enu_yaw_get_x(msg);
    uav_loc_pos_enu_yaw->y = mavlink_msg_uav_loc_pos_enu_yaw_get_y(msg);
    uav_loc_pos_enu_yaw->z = mavlink_msg_uav_loc_pos_enu_yaw_get_z(msg);
    uav_loc_pos_enu_yaw->yaw = mavlink_msg_uav_loc_pos_enu_yaw_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN? msg->len : MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN;
        memset(uav_loc_pos_enu_yaw, 0, MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW_LEN);
    memcpy(uav_loc_pos_enu_yaw, _MAV_PAYLOAD(msg), len);
#endif
}
