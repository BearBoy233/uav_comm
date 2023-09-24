#pragma once
// MESSAGE PX4_SPECIAL_SET PACKING

#define MAVLINK_MSG_ID_PX4_SPECIAL_SET 6125


typedef struct __mavlink_px4_special_set_t {
 uint8_t param1; /*<  px4_special_set param1.*/
 uint8_t param2; /*<  px4_special_set param2.*/
} mavlink_px4_special_set_t;

#define MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN 2
#define MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN 2
#define MAVLINK_MSG_ID_6125_LEN 2
#define MAVLINK_MSG_ID_6125_MIN_LEN 2

#define MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC 191
#define MAVLINK_MSG_ID_6125_CRC 191



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4_SPECIAL_SET { \
    6125, \
    "PX4_SPECIAL_SET", \
    2, \
    {  { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4_special_set_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4_special_set_t, param2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4_SPECIAL_SET { \
    "PX4_SPECIAL_SET", \
    2, \
    {  { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4_special_set_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4_special_set_t, param2) }, \
         } \
}
#endif

/**
 * @brief Pack a px4_special_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param1  px4_special_set param1.
 * @param param2  px4_special_set param2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_special_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN);
#else
    mavlink_px4_special_set_t packet;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_SPECIAL_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
}

/**
 * @brief Pack a px4_special_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param1  px4_special_set param1.
 * @param param2  px4_special_set param2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_special_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t param1,uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN);
#else
    mavlink_px4_special_set_t packet;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_SPECIAL_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
}

/**
 * @brief Encode a px4_special_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_special_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_special_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_special_set_t* px4_special_set)
{
    return mavlink_msg_px4_special_set_pack(system_id, component_id, msg, px4_special_set->param1, px4_special_set->param2);
}

/**
 * @brief Encode a px4_special_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_special_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_special_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_special_set_t* px4_special_set)
{
    return mavlink_msg_px4_special_set_pack_chan(system_id, component_id, chan, msg, px4_special_set->param1, px4_special_set->param2);
}

/**
 * @brief Send a px4_special_set message
 * @param chan MAVLink channel to send the message
 *
 * @param param1  px4_special_set param1.
 * @param param2  px4_special_set param2.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_special_set_send(mavlink_channel_t chan, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET, buf, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
#else
    mavlink_px4_special_set_t packet;
    packet.param1 = param1;
    packet.param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET, (const char *)&packet, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
#endif
}

/**
 * @brief Send a px4_special_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4_special_set_send_struct(mavlink_channel_t chan, const mavlink_px4_special_set_t* px4_special_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4_special_set_send(chan, px4_special_set->param1, px4_special_set->param2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET, (const char *)px4_special_set, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_special_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET, buf, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
#else
    mavlink_px4_special_set_t *packet = (mavlink_px4_special_set_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_SET, (const char *)packet, MAVLINK_MSG_ID_PX4_SPECIAL_SET_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4_SPECIAL_SET UNPACKING


/**
 * @brief Get field param1 from px4_special_set message
 *
 * @return  px4_special_set param1.
 */
static inline uint8_t mavlink_msg_px4_special_set_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param2 from px4_special_set message
 *
 * @return  px4_special_set param2.
 */
static inline uint8_t mavlink_msg_px4_special_set_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a px4_special_set message into a struct
 *
 * @param msg The message to decode
 * @param px4_special_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_special_set_decode(const mavlink_message_t* msg, mavlink_px4_special_set_t* px4_special_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4_special_set->param1 = mavlink_msg_px4_special_set_get_param1(msg);
    px4_special_set->param2 = mavlink_msg_px4_special_set_get_param2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN? msg->len : MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN;
        memset(px4_special_set, 0, MAVLINK_MSG_ID_PX4_SPECIAL_SET_LEN);
    memcpy(px4_special_set, _MAV_PAYLOAD(msg), len);
#endif
}
