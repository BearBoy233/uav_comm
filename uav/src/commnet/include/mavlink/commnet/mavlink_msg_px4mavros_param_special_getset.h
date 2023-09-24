#pragma once
// MESSAGE PX4MAVROS_PARAM_SPECIAL_GETSET PACKING

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET 6024


typedef struct __mavlink_px4mavros_param_special_getset_t {
 uint8_t flag; /*<   标志位 1-重启飞控, 2-获取无人机的定位设置参数, 3-设置无人机的定位.*/
 uint8_t param; /*<   配合部分flag.*/
} mavlink_px4mavros_param_special_getset_t;

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN 2
#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN 2
#define MAVLINK_MSG_ID_6024_LEN 2
#define MAVLINK_MSG_ID_6024_MIN_LEN 2

#define MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC 190
#define MAVLINK_MSG_ID_6024_CRC 190



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4MAVROS_PARAM_SPECIAL_GETSET { \
    6024, \
    "PX4MAVROS_PARAM_SPECIAL_GETSET", \
    2, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4mavros_param_special_getset_t, flag) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4mavros_param_special_getset_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4MAVROS_PARAM_SPECIAL_GETSET { \
    "PX4MAVROS_PARAM_SPECIAL_GETSET", \
    2, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_px4mavros_param_special_getset_t, flag) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_px4mavros_param_special_getset_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a px4mavros_param_special_getset message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flag   标志位 1-重启飞控, 2-获取无人机的定位设置参数, 3-设置无人机的定位.
 * @param param   配合部分flag.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_getset_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN);
#else
    mavlink_px4mavros_param_special_getset_t packet;
    packet.flag = flag;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
}

/**
 * @brief Pack a px4mavros_param_special_getset message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flag   标志位 1-重启飞控, 2-获取无人机的定位设置参数, 3-设置无人机的定位.
 * @param param   配合部分flag.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_getset_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t flag,uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN);
#else
    mavlink_px4mavros_param_special_getset_t packet;
    packet.flag = flag;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
}

/**
 * @brief Encode a px4mavros_param_special_getset struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4mavros_param_special_getset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_getset_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4mavros_param_special_getset_t* px4mavros_param_special_getset)
{
    return mavlink_msg_px4mavros_param_special_getset_pack(system_id, component_id, msg, px4mavros_param_special_getset->flag, px4mavros_param_special_getset->param);
}

/**
 * @brief Encode a px4mavros_param_special_getset struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4mavros_param_special_getset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4mavros_param_special_getset_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4mavros_param_special_getset_t* px4mavros_param_special_getset)
{
    return mavlink_msg_px4mavros_param_special_getset_pack_chan(system_id, component_id, chan, msg, px4mavros_param_special_getset->flag, px4mavros_param_special_getset->param);
}

/**
 * @brief Send a px4mavros_param_special_getset message
 * @param chan MAVLink channel to send the message
 *
 * @param flag   标志位 1-重启飞控, 2-获取无人机的定位设置参数, 3-设置无人机的定位.
 * @param param   配合部分flag.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4mavros_param_special_getset_send(mavlink_channel_t chan, uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET, buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
#else
    mavlink_px4mavros_param_special_getset_t packet;
    packet.flag = flag;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET, (const char *)&packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
#endif
}

/**
 * @brief Send a px4mavros_param_special_getset message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4mavros_param_special_getset_send_struct(mavlink_channel_t chan, const mavlink_px4mavros_param_special_getset_t* px4mavros_param_special_getset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4mavros_param_special_getset_send(chan, px4mavros_param_special_getset->flag, px4mavros_param_special_getset->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET, (const char *)px4mavros_param_special_getset, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4mavros_param_special_getset_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET, buf, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
#else
    mavlink_px4mavros_param_special_getset_t *packet = (mavlink_px4mavros_param_special_getset_t *)msgbuf;
    packet->flag = flag;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET, (const char *)packet, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_MIN_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4MAVROS_PARAM_SPECIAL_GETSET UNPACKING


/**
 * @brief Get field flag from px4mavros_param_special_getset message
 *
 * @return   标志位 1-重启飞控, 2-获取无人机的定位设置参数, 3-设置无人机的定位.
 */
static inline uint8_t mavlink_msg_px4mavros_param_special_getset_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from px4mavros_param_special_getset message
 *
 * @return   配合部分flag.
 */
static inline uint8_t mavlink_msg_px4mavros_param_special_getset_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a px4mavros_param_special_getset message into a struct
 *
 * @param msg The message to decode
 * @param px4mavros_param_special_getset C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4mavros_param_special_getset_decode(const mavlink_message_t* msg, mavlink_px4mavros_param_special_getset_t* px4mavros_param_special_getset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4mavros_param_special_getset->flag = mavlink_msg_px4mavros_param_special_getset_get_flag(msg);
    px4mavros_param_special_getset->param = mavlink_msg_px4mavros_param_special_getset_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN? msg->len : MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN;
        memset(px4mavros_param_special_getset, 0, MAVLINK_MSG_ID_PX4MAVROS_PARAM_SPECIAL_GETSET_LEN);
    memcpy(px4mavros_param_special_getset, _MAV_PAYLOAD(msg), len);
#endif
}
