#pragma once
// MESSAGE PX4MAVROS_PARAM_SPECIAL_BACK PACKING

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK 6025


typedef struct __mavlink_px4mavros_param_special_back_t {
 uint8_t msg_EKF2_AID_MASK; /*<   EKF2_AID_MASK [1-use GPS, 24-use vision].*/
 uint8_t msg_EKF2_HGT_MODE; /*<   EKF2_HGT_MODE [0-barometric pressure, 1-GPS, 3-Vision].*/
} mavlink_px4mavros_param_special_back_t;

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN 2
#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN 2
#define MAVLINK_MSG_ID_6025_LEN 2
#define MAVLINK_MSG_ID_6025_MIN_LEN 2

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC 52
#define MAVLINK_MSG_ID_6025_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4MAVROS_PARAM_SPECIAL_BACK { \
    6025, \
    "PX4MAVROS_PARAM_SPECIAL_BACK", \
    2, \
    {  { "msg_EKF2_AID_MASK", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4mavros_param_special_back_t, msg_EKF2_AID_MASK) }, \
         { "msg_EKF2_HGT_MODE", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4mavros_param_special_back_t, msg_EKF2_HGT_MODE) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4MAVROS_PARAM_SPECIAL_BACK { \
    "PX4MAVROS_PARAM_SPECIAL_BACK", \
    2, \
    {  { "msg_EKF2_AID_MASK", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4mavros_param_special_back_t, msg_EKF2_AID_MASK) }, \
         { "msg_EKF2_HGT_MODE", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4mavros_param_special_back_t, msg_EKF2_HGT_MODE) }, \
         } \
}
#endif

/**
 * @brief Pack a px4mavros_param_special_back message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msg_EKF2_AID_MASK   EKF2_AID_MASK [1-use GPS, 24-use vision].
 * @param msg_EKF2_HGT_MODE   EKF2_HGT_MODE [0-barometric pressure, 1-GPS, 3-Vision].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_back_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN);
#else
    mavlink_px4mavros_param_special_back_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
}

/**
 * @brief Pack a px4mavros_param_special_back message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param msg_EKF2_AID_MASK   EKF2_AID_MASK [1-use GPS, 24-use vision].
 * @param msg_EKF2_HGT_MODE   EKF2_HGT_MODE [0-barometric pressure, 1-GPS, 3-Vision].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_back_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t msg_EKF2_AID_MASK,uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN);
#else
    mavlink_px4mavros_param_special_back_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
}

/**
 * @brief Encode a px4mavros_param_special_back struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4mavros_param_special_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_back_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4mavros_param_special_back_t* px4mavros_param_special_back)
{
    return mavlink_msg_px4mavros_param_special_back_pack(system_id, component_id, msg, px4mavros_param_special_back->msg_EKF2_AID_MASK, px4mavros_param_special_back->msg_EKF2_HGT_MODE);
}

/**
 * @brief Encode a px4mavros_param_special_back struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4mavros_param_special_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_back_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4mavros_param_special_back_t* px4mavros_param_special_back)
{
    return mavlink_msg_px4mavros_param_special_back_pack_chan(system_id, component_id, chan, msg, px4mavros_param_special_back->msg_EKF2_AID_MASK, px4mavros_param_special_back->msg_EKF2_HGT_MODE);
}

/**
 * @brief Send a px4mavros_param_special_back message
 * @param chan MAVLink channel to send the message
 *
 * @param msg_EKF2_AID_MASK   EKF2_AID_MASK [1-use GPS, 24-use vision].
 * @param msg_EKF2_HGT_MODE   EKF2_HGT_MODE [0-barometric pressure, 1-GPS, 3-Vision].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4mavros_param_special_back_send(mavlink_channel_t chan, uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN];
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK, buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
#else
    mavlink_px4mavros_param_special_back_t packet;
    packet.msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet.msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK, (const char *)&packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
#endif
}

/**
 * @brief Send a px4mavros_param_special_back message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4mavros_param_special_back_send_struct(mavlink_channel_t chan, const mavlink_px4mavros_param_special_back_t* px4mavros_param_special_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4mavros_param_special_back_send(chan, px4mavros_param_special_back->msg_EKF2_AID_MASK, px4mavros_param_special_back->msg_EKF2_HGT_MODE);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK, (const char *)px4mavros_param_special_back, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4mavros_param_special_back_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t msg_EKF2_AID_MASK, uint8_t msg_EKF2_HGT_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, msg_EKF2_AID_MASK);
    _mav_put_uint8_t(buf, 1, msg_EKF2_HGT_MODE);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK, buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
#else
    mavlink_px4mavros_param_special_back_t *packet = (mavlink_px4mavros_param_special_back_t *)msgbuf;
    packet->msg_EKF2_AID_MASK = msg_EKF2_AID_MASK;
    packet->msg_EKF2_HGT_MODE = msg_EKF2_HGT_MODE;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK, (const char *)packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4MAVROS_PARAM_SPECIAL_BACK UNPACKING


/**
 * @brief Get field msg_EKF2_AID_MASK from px4mavros_param_special_back message
 *
 * @return   EKF2_AID_MASK [1-use GPS, 24-use vision].
 */
static inline uint8_t mavlink_msg_px4mavros_param_special_back_get_msg_EKF2_AID_MASK(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field msg_EKF2_HGT_MODE from px4mavros_param_special_back message
 *
 * @return   EKF2_HGT_MODE [0-barometric pressure, 1-GPS, 3-Vision].
 */
static inline uint8_t mavlink_msg_px4mavros_param_special_back_get_msg_EKF2_HGT_MODE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a px4mavros_param_special_back message into a struct
 *
 * @param msg The message to decode
 * @param px4mavros_param_special_back C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4mavros_param_special_back_decode(const mavlink_message_t* msg, mavlink_px4mavros_param_special_back_t* px4mavros_param_special_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4mavros_param_special_back->msg_EKF2_AID_MASK = mavlink_msg_px4mavros_param_special_back_get_msg_EKF2_AID_MASK(msg);
    px4mavros_param_special_back->msg_EKF2_HGT_MODE = mavlink_msg_px4mavros_param_special_back_get_msg_EKF2_HGT_MODE(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN? msg->len : MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN;
        memset(px4mavros_param_special_back, 0, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_BACK_LEN);
    memcpy(px4mavros_param_special_back, _MAV_PAYLOAD(msg), len);
#endif
}
