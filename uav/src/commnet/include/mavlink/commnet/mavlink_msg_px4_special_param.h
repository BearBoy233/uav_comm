#pragma once
// MESSAGE PX4_SPECIAL_PARAM PACKING

#define MAVLINK_MSG_ID_PX4_SPECIAL_PARAM 6124


typedef struct __mavlink_px4_special_param_t {
 uint8_t msg_EKF2_AID_MASK; /*<  px4_special_param msg_EKF2_AID_MASK.*/
 uint8_t msg_EKF2_HGT_MODE; /*<  px4_special_param msg_EKF2_HGT_MODE.*/
} mavlink_px4_special_param_t;

#define MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN 2
#define MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN 2
#define MAVLINK_MSG_ID_6124_LEN 2
#define MAVLINK_MSG_ID_6124_MIN_LEN 2

#define MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC 9
#define MAVLINK_MSG_ID_6124_CRC 9



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4_SPECIAL_PARAM { \
    6124, \
    "PX4_SPECIAL_PARAM", \
    2, \
    {  { "msg_EKF2_AID_MASK", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4_special_param_t, msg_EKF2_AID_MASK) }, \
         { "msg_EKF2_HGT_MODE", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4_special_param_t, msg_EKF2_HGT_MODE) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4_SPECIAL_PARAM { \
    "PX4_SPECIAL_PARAM", \
    2, \
    {  { "msg_EKF2_AID_MASK", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4_special_param_t, msg_EKF2_AID_MASK) }, \
         { "msg_EKF2_HGT_MODE", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4_special_param_t, msg_EKF2_HGT_MODE) }, \
         } \
}
#endif

/**
 * @brief Pack a px4_special_param message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msg_EKF2_AID_MASK  px4_special_param msg_EKF2_AID_MASK.
 * @param msg_EKF2_HGT_MODE  px4_special_param msg_EKF2_HGT_MODE.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_special_param_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN);
#else
    mavlink_px4_special_param_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_SPECIAL_PARAM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
}

/**
 * @brief Pack a px4_special_param message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param msg_EKF2_AID_MASK  px4_special_param msg_EKF2_AID_MASK.
 * @param msg_EKF2_HGT_MODE  px4_special_param msg_EKF2_HGT_MODE.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_special_param_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t msg_EKF2_AID_MASK,uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN);
#else
    mavlink_px4_special_param_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_SPECIAL_PARAM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
}

/**
 * @brief Encode a px4_special_param struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_special_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_special_param_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_special_param_t* px4_special_param)
{
    return mavlink_msg_px4_special_param_pack(system_id, component_id, msg, px4_special_param->msg_EKF2_AID_MASK, px4_special_param->msg_EKF2_HGT_MODE);
}

/**
 * @brief Encode a px4_special_param struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_special_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_special_param_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_special_param_t* px4_special_param)
{
    return mavlink_msg_px4_special_param_pack_chan(system_id, component_id, chan, msg, px4_special_param->msg_EKF2_AID_MASK, px4_special_param->msg_EKF2_HGT_MODE);
}

/**
 * @brief Send a px4_special_param message
 * @param chan MAVLink channel to send the message
 *
 * @param msg_EKF2_AID_MASK  px4_special_param msg_EKF2_AID_MASK.
 * @param msg_EKF2_HGT_MODE  px4_special_param msg_EKF2_HGT_MODE.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_special_param_send(mavlink_channel_t chan, uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM, buf, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
#else
    mavlink_px4_special_param_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM, (const char *)&packet, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
#endif
}

/**
 * @brief Send a px4_special_param message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4_special_param_send_struct(mavlink_channel_t chan, const mavlink_px4_special_param_t* px4_special_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4_special_param_send(chan, px4_special_param->msg_EKF2_AID_MASK, px4_special_param->msg_EKF2_HGT_MODE);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM, (const char *)px4_special_param, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_special_param_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM, buf, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
#else
    mavlink_px4_special_param_t *packet = (mavlink_px4_special_param_t *)msgbuf;
    packet->msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet->msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM, (const char *)packet, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_MIN_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4_SPECIAL_PARAM UNPACKING


/**
 * @brief Get field msg_EKF2_AID_MASK from px4_special_param message
 *
 * @return  px4_special_param msg_EKF2_AID_MASK.
 */
static inline uint8_t mavlink_msg_px4_special_param_get_msg_EKF2_AID_MASK(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field msg_EKF2_HGT_MODE from px4_special_param message
 *
 * @return  px4_special_param msg_EKF2_HGT_MODE.
 */
static inline uint8_t mavlink_msg_px4_special_param_get_msg_EKF2_HGT_MODE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a px4_special_param message into a struct
 *
 * @param msg The message to decode
 * @param px4_special_param C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_special_param_decode(const mavlink_message_t* msg, mavlink_px4_special_param_t* px4_special_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4_special_param->msg_EKF2_AID_MASK = mavlink_msg_px4_special_param_get_msg_EKF2_AID_MASK(msg);
    px4_special_param->msg_EKF2_HGT_MODE = mavlink_msg_px4_special_param_get_msg_EKF2_HGT_MODE(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN? msg->len : MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN;
        memset(px4_special_param, 0, MAVLINK_MSG_ID_PX4_SPECIAL_PARAM_LEN);
    memcpy(px4_special_param, _MAV_PAYLOAD(msg), len);
#endif
}
