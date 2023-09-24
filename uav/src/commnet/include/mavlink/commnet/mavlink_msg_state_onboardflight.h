#pragma once
// MESSAGE STATE_ONBOARDFLIGHT PACKING

#define MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT 5501


typedef struct __mavlink_state_onboardflight_t {
 uint8_t node_state; /*<  切换状态.*/
 uint8_t param; /*<   To be used later */
} mavlink_state_onboardflight_t;

#define MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN 2
#define MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN 2
#define MAVLINK_MSG_ID_5501_LEN 2
#define MAVLINK_MSG_ID_5501_MIN_LEN 2

#define MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC 205
#define MAVLINK_MSG_ID_5501_CRC 205



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATE_ONBOARDFLIGHT { \
    5501, \
    "STATE_ONBOARDFLIGHT", \
    2, \
    {  { "node_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_onboardflight_t, node_state) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_onboardflight_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATE_ONBOARDFLIGHT { \
    "STATE_ONBOARDFLIGHT", \
    2, \
    {  { "node_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_onboardflight_t, node_state) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_onboardflight_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a state_onboardflight message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param node_state  切换状态.
 * @param param   To be used later 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_onboardflight_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t node_state, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN];
    _mav_put_uint8_t(buf, 0, node_state);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN);
#else
    mavlink_state_onboardflight_t packet;
    packet.node_state = node_state;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
}

/**
 * @brief Pack a state_onboardflight message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_state  切换状态.
 * @param param   To be used later 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_onboardflight_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t node_state,uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN];
    _mav_put_uint8_t(buf, 0, node_state);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN);
#else
    mavlink_state_onboardflight_t packet;
    packet.node_state = node_state;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
}

/**
 * @brief Encode a state_onboardflight struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_onboardflight C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_onboardflight_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_onboardflight_t* state_onboardflight)
{
    return mavlink_msg_state_onboardflight_pack(system_id, component_id, msg, state_onboardflight->node_state, state_onboardflight->param);
}

/**
 * @brief Encode a state_onboardflight struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state_onboardflight C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_onboardflight_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_state_onboardflight_t* state_onboardflight)
{
    return mavlink_msg_state_onboardflight_pack_chan(system_id, component_id, chan, msg, state_onboardflight->node_state, state_onboardflight->param);
}

/**
 * @brief Send a state_onboardflight message
 * @param chan MAVLink channel to send the message
 *
 * @param node_state  切换状态.
 * @param param   To be used later 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_onboardflight_send(mavlink_channel_t chan, uint8_t node_state, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN];
    _mav_put_uint8_t(buf, 0, node_state);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT, buf, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
#else
    mavlink_state_onboardflight_t packet;
    packet.node_state = node_state;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT, (const char *)&packet, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
#endif
}

/**
 * @brief Send a state_onboardflight message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_state_onboardflight_send_struct(mavlink_channel_t chan, const mavlink_state_onboardflight_t* state_onboardflight)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_state_onboardflight_send(chan, state_onboardflight->node_state, state_onboardflight->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT, (const char *)state_onboardflight, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_state_onboardflight_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t node_state, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, node_state);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT, buf, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
#else
    mavlink_state_onboardflight_t *packet = (mavlink_state_onboardflight_t *)msgbuf;
    packet->node_state = node_state;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT, (const char *)packet, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_MIN_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_CRC);
#endif
}
#endif

#endif

// MESSAGE STATE_ONBOARDFLIGHT UNPACKING


/**
 * @brief Get field node_state from state_onboardflight message
 *
 * @return  切换状态.
 */
static inline uint8_t mavlink_msg_state_onboardflight_get_node_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from state_onboardflight message
 *
 * @return   To be used later 
 */
static inline uint8_t mavlink_msg_state_onboardflight_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a state_onboardflight message into a struct
 *
 * @param msg The message to decode
 * @param state_onboardflight C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_onboardflight_decode(const mavlink_message_t* msg, mavlink_state_onboardflight_t* state_onboardflight)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    state_onboardflight->node_state = mavlink_msg_state_onboardflight_get_node_state(msg);
    state_onboardflight->param = mavlink_msg_state_onboardflight_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN? msg->len : MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN;
        memset(state_onboardflight, 0, MAVLINK_MSG_ID_STATE_ONBOARDFLIGHT_LEN);
    memcpy(state_onboardflight, _MAV_PAYLOAD(msg), len);
#endif
}
