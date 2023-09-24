#pragma once
// MESSAGE MISSION_EXEC PACKING

#define MAVLINK_MSG_ID_MISSION_EXEC 810


typedef struct __mavlink_mission_exec_t {
 uint8_t instruct_status; /*<  instruct_status.*/
 uint8_t mission_no; /*<  mission_no.*/
 uint8_t flag; /*<  flag.*/
 uint8_t param; /*<  param.*/
} mavlink_mission_exec_t;

#define MAVLINK_MSG_ID_MISSION_EXEC_LEN 4
#define MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN 4
#define MAVLINK_MSG_ID_810_LEN 4
#define MAVLINK_MSG_ID_810_MIN_LEN 4

#define MAVLINK_MSG_ID_MISSION_EXEC_CRC 142
#define MAVLINK_MSG_ID_810_CRC 142



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_EXEC { \
    810, \
    "MISSION_EXEC", \
    4, \
    {  { "instruct_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_exec_t, instruct_status) }, \
         { "mission_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_exec_t, mission_no) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_exec_t, flag) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_exec_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_EXEC { \
    "MISSION_EXEC", \
    4, \
    {  { "instruct_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_exec_t, instruct_status) }, \
         { "mission_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_exec_t, mission_no) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_exec_t, flag) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_exec_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_exec message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param instruct_status  instruct_status.
 * @param mission_no  mission_no.
 * @param flag  flag.
 * @param param  param.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_exec_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t instruct_status, uint8_t mission_no, uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_EXEC_LEN];
    _mav_put_uint8_t(buf, 0, instruct_status);
    _mav_put_uint8_t(buf, 1, mission_no);
    _mav_put_uint8_t(buf, 2, flag);
    _mav_put_uint8_t(buf, 3, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_EXEC_LEN);
#else
    mavlink_mission_exec_t packet;
    packet.instruct_status = instruct_status;
    packet.mission_no = mission_no;
    packet.flag = flag;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_EXEC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_EXEC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
}

/**
 * @brief Pack a mission_exec message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param instruct_status  instruct_status.
 * @param mission_no  mission_no.
 * @param flag  flag.
 * @param param  param.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_exec_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t instruct_status,uint8_t mission_no,uint8_t flag,uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_EXEC_LEN];
    _mav_put_uint8_t(buf, 0, instruct_status);
    _mav_put_uint8_t(buf, 1, mission_no);
    _mav_put_uint8_t(buf, 2, flag);
    _mav_put_uint8_t(buf, 3, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_EXEC_LEN);
#else
    mavlink_mission_exec_t packet;
    packet.instruct_status = instruct_status;
    packet.mission_no = mission_no;
    packet.flag = flag;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_EXEC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_EXEC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
}

/**
 * @brief Encode a mission_exec struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_exec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_exec_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_exec_t* mission_exec)
{
    return mavlink_msg_mission_exec_pack(system_id, component_id, msg, mission_exec->instruct_status, mission_exec->mission_no, mission_exec->flag, mission_exec->param);
}

/**
 * @brief Encode a mission_exec struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_exec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_exec_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_exec_t* mission_exec)
{
    return mavlink_msg_mission_exec_pack_chan(system_id, component_id, chan, msg, mission_exec->instruct_status, mission_exec->mission_no, mission_exec->flag, mission_exec->param);
}

/**
 * @brief Send a mission_exec message
 * @param chan MAVLink channel to send the message
 *
 * @param instruct_status  instruct_status.
 * @param mission_no  mission_no.
 * @param flag  flag.
 * @param param  param.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_exec_send(mavlink_channel_t chan, uint8_t instruct_status, uint8_t mission_no, uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_EXEC_LEN];
    _mav_put_uint8_t(buf, 0, instruct_status);
    _mav_put_uint8_t(buf, 1, mission_no);
    _mav_put_uint8_t(buf, 2, flag);
    _mav_put_uint8_t(buf, 3, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_EXEC, buf, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
#else
    mavlink_mission_exec_t packet;
    packet.instruct_status = instruct_status;
    packet.mission_no = mission_no;
    packet.flag = flag;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_EXEC, (const char *)&packet, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
#endif
}

/**
 * @brief Send a mission_exec message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_exec_send_struct(mavlink_channel_t chan, const mavlink_mission_exec_t* mission_exec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_exec_send(chan, mission_exec->instruct_status, mission_exec->mission_no, mission_exec->flag, mission_exec->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_EXEC, (const char *)mission_exec, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_EXEC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_exec_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t instruct_status, uint8_t mission_no, uint8_t flag, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, instruct_status);
    _mav_put_uint8_t(buf, 1, mission_no);
    _mav_put_uint8_t(buf, 2, flag);
    _mav_put_uint8_t(buf, 3, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_EXEC, buf, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
#else
    mavlink_mission_exec_t *packet = (mavlink_mission_exec_t *)msgbuf;
    packet->instruct_status = instruct_status;
    packet->mission_no = mission_no;
    packet->flag = flag;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_EXEC, (const char *)packet, MAVLINK_MSG_ID_MISSION_EXEC_MIN_LEN, MAVLINK_MSG_ID_MISSION_EXEC_LEN, MAVLINK_MSG_ID_MISSION_EXEC_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_EXEC UNPACKING


/**
 * @brief Get field instruct_status from mission_exec message
 *
 * @return  instruct_status.
 */
static inline uint8_t mavlink_msg_mission_exec_get_instruct_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_no from mission_exec message
 *
 * @return  mission_no.
 */
static inline uint8_t mavlink_msg_mission_exec_get_mission_no(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field flag from mission_exec message
 *
 * @return  flag.
 */
static inline uint8_t mavlink_msg_mission_exec_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field param from mission_exec message
 *
 * @return  param.
 */
static inline uint8_t mavlink_msg_mission_exec_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mission_exec message into a struct
 *
 * @param msg The message to decode
 * @param mission_exec C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_exec_decode(const mavlink_message_t* msg, mavlink_mission_exec_t* mission_exec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_exec->instruct_status = mavlink_msg_mission_exec_get_instruct_status(msg);
    mission_exec->mission_no = mavlink_msg_mission_exec_get_mission_no(msg);
    mission_exec->flag = mavlink_msg_mission_exec_get_flag(msg);
    mission_exec->param = mavlink_msg_mission_exec_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_EXEC_LEN? msg->len : MAVLINK_MSG_ID_MISSION_EXEC_LEN;
        memset(mission_exec, 0, MAVLINK_MSG_ID_MISSION_EXEC_LEN);
    memcpy(mission_exec, _MAV_PAYLOAD(msg), len);
#endif
}
