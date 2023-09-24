#pragma once
// MESSAGE MISSION_SET PACKING

#define MAVLINK_MSG_ID_MISSION_SET 802


typedef struct __mavlink_mission_set_t {
 float x; /*<  (m) pos x.*/
 float y; /*<  (m) pos y.*/
 float z; /*<  (m) pos z.*/
 float yaw; /*<  (rad) yaw.*/
 uint16_t uav_no_16; /*<  uav no 16.*/
 uint8_t mission_no; /*<  mission number.*/
 uint8_t mission_task; /*<  mission task.*/
 uint8_t flag; /*<  flag.*/
 uint8_t param1; /*<  param1.*/
 uint8_t param2; /*<  param2.*/
 uint8_t param3; /*<  param3.*/
} mavlink_mission_set_t;

#define MAVLINK_MSG_ID_MISSION_SET_LEN 24
#define MAVLINK_MSG_ID_MISSION_SET_MIN_LEN 24
#define MAVLINK_MSG_ID_802_LEN 24
#define MAVLINK_MSG_ID_802_MIN_LEN 24

#define MAVLINK_MSG_ID_MISSION_SET_CRC 245
#define MAVLINK_MSG_ID_802_CRC 245



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_SET { \
    802, \
    "MISSION_SET", \
    11, \
    {  { "mission_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_mission_set_t, mission_no) }, \
         { "uav_no_16", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_mission_set_t, uav_no_16) }, \
         { "mission_task", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_mission_set_t, mission_task) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mission_set_t, flag) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mission_set_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mission_set_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_set_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_set_t, yaw) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_mission_set_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mission_set_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_mission_set_t, param3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_SET { \
    "MISSION_SET", \
    11, \
    {  { "mission_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_mission_set_t, mission_no) }, \
         { "uav_no_16", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_mission_set_t, uav_no_16) }, \
         { "mission_task", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_mission_set_t, mission_task) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mission_set_t, flag) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mission_set_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mission_set_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_set_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_set_t, yaw) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_mission_set_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mission_set_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_mission_set_t, param3) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_no  mission number.
 * @param uav_no_16  uav no 16.
 * @param mission_task  mission task.
 * @param flag  flag.
 * @param x  (m) pos x.
 * @param y  (m) pos y.
 * @param z  (m) pos z.
 * @param yaw  (rad) yaw.
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mission_no, uint16_t uav_no_16, uint8_t mission_task, uint8_t flag, float x, float y, float z, float yaw, uint8_t param1, uint8_t param2, uint8_t param3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_SET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, uav_no_16);
    _mav_put_uint8_t(buf, 18, mission_no);
    _mav_put_uint8_t(buf, 19, mission_task);
    _mav_put_uint8_t(buf, 20, flag);
    _mav_put_uint8_t(buf, 21, param1);
    _mav_put_uint8_t(buf, 22, param2);
    _mav_put_uint8_t(buf, 23, param3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_SET_LEN);
#else
    mavlink_mission_set_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.uav_no_16 = uav_no_16;
    packet.mission_no = mission_no;
    packet.mission_task = mission_task;
    packet.flag = flag;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
}

/**
 * @brief Pack a mission_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_no  mission number.
 * @param uav_no_16  uav no 16.
 * @param mission_task  mission task.
 * @param flag  flag.
 * @param x  (m) pos x.
 * @param y  (m) pos y.
 * @param z  (m) pos z.
 * @param yaw  (rad) yaw.
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mission_no,uint16_t uav_no_16,uint8_t mission_task,uint8_t flag,float x,float y,float z,float yaw,uint8_t param1,uint8_t param2,uint8_t param3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_SET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, uav_no_16);
    _mav_put_uint8_t(buf, 18, mission_no);
    _mav_put_uint8_t(buf, 19, mission_task);
    _mav_put_uint8_t(buf, 20, flag);
    _mav_put_uint8_t(buf, 21, param1);
    _mav_put_uint8_t(buf, 22, param2);
    _mav_put_uint8_t(buf, 23, param3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_SET_LEN);
#else
    mavlink_mission_set_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.uav_no_16 = uav_no_16;
    packet.mission_no = mission_no;
    packet.mission_task = mission_task;
    packet.flag = flag;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
}

/**
 * @brief Encode a mission_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_set_t* mission_set)
{
    return mavlink_msg_mission_set_pack(system_id, component_id, msg, mission_set->mission_no, mission_set->uav_no_16, mission_set->mission_task, mission_set->flag, mission_set->x, mission_set->y, mission_set->z, mission_set->yaw, mission_set->param1, mission_set->param2, mission_set->param3);
}

/**
 * @brief Encode a mission_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_set_t* mission_set)
{
    return mavlink_msg_mission_set_pack_chan(system_id, component_id, chan, msg, mission_set->mission_no, mission_set->uav_no_16, mission_set->mission_task, mission_set->flag, mission_set->x, mission_set->y, mission_set->z, mission_set->yaw, mission_set->param1, mission_set->param2, mission_set->param3);
}

/**
 * @brief Send a mission_set message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_no  mission number.
 * @param uav_no_16  uav no 16.
 * @param mission_task  mission task.
 * @param flag  flag.
 * @param x  (m) pos x.
 * @param y  (m) pos y.
 * @param z  (m) pos z.
 * @param yaw  (rad) yaw.
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_set_send(mavlink_channel_t chan, uint8_t mission_no, uint16_t uav_no_16, uint8_t mission_task, uint8_t flag, float x, float y, float z, float yaw, uint8_t param1, uint8_t param2, uint8_t param3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_SET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, uav_no_16);
    _mav_put_uint8_t(buf, 18, mission_no);
    _mav_put_uint8_t(buf, 19, mission_task);
    _mav_put_uint8_t(buf, 20, flag);
    _mav_put_uint8_t(buf, 21, param1);
    _mav_put_uint8_t(buf, 22, param2);
    _mav_put_uint8_t(buf, 23, param3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_SET, buf, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
#else
    mavlink_mission_set_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.uav_no_16 = uav_no_16;
    packet.mission_no = mission_no;
    packet.mission_task = mission_task;
    packet.flag = flag;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_SET, (const char *)&packet, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
#endif
}

/**
 * @brief Send a mission_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_set_send_struct(mavlink_channel_t chan, const mavlink_mission_set_t* mission_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_set_send(chan, mission_set->mission_no, mission_set->uav_no_16, mission_set->mission_task, mission_set->flag, mission_set->x, mission_set->y, mission_set->z, mission_set->yaw, mission_set->param1, mission_set->param2, mission_set->param3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_SET, (const char *)mission_set, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mission_no, uint16_t uav_no_16, uint8_t mission_task, uint8_t flag, float x, float y, float z, float yaw, uint8_t param1, uint8_t param2, uint8_t param3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, uav_no_16);
    _mav_put_uint8_t(buf, 18, mission_no);
    _mav_put_uint8_t(buf, 19, mission_task);
    _mav_put_uint8_t(buf, 20, flag);
    _mav_put_uint8_t(buf, 21, param1);
    _mav_put_uint8_t(buf, 22, param2);
    _mav_put_uint8_t(buf, 23, param3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_SET, buf, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
#else
    mavlink_mission_set_t *packet = (mavlink_mission_set_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->uav_no_16 = uav_no_16;
    packet->mission_no = mission_no;
    packet->mission_task = mission_task;
    packet->flag = flag;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_SET, (const char *)packet, MAVLINK_MSG_ID_MISSION_SET_MIN_LEN, MAVLINK_MSG_ID_MISSION_SET_LEN, MAVLINK_MSG_ID_MISSION_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_SET UNPACKING


/**
 * @brief Get field mission_no from mission_set message
 *
 * @return  mission number.
 */
static inline uint8_t mavlink_msg_mission_set_get_mission_no(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field uav_no_16 from mission_set message
 *
 * @return  uav no 16.
 */
static inline uint16_t mavlink_msg_mission_set_get_uav_no_16(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field mission_task from mission_set message
 *
 * @return  mission task.
 */
static inline uint8_t mavlink_msg_mission_set_get_mission_task(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field flag from mission_set message
 *
 * @return  flag.
 */
static inline uint8_t mavlink_msg_mission_set_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field x from mission_set message
 *
 * @return  (m) pos x.
 */
static inline float mavlink_msg_mission_set_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from mission_set message
 *
 * @return  (m) pos y.
 */
static inline float mavlink_msg_mission_set_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from mission_set message
 *
 * @return  (m) pos z.
 */
static inline float mavlink_msg_mission_set_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from mission_set message
 *
 * @return  (rad) yaw.
 */
static inline float mavlink_msg_mission_set_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field param1 from mission_set message
 *
 * @return  param1.
 */
static inline uint8_t mavlink_msg_mission_set_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field param2 from mission_set message
 *
 * @return  param2.
 */
static inline uint8_t mavlink_msg_mission_set_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field param3 from mission_set message
 *
 * @return  param3.
 */
static inline uint8_t mavlink_msg_mission_set_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Decode a mission_set message into a struct
 *
 * @param msg The message to decode
 * @param mission_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_set_decode(const mavlink_message_t* msg, mavlink_mission_set_t* mission_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_set->x = mavlink_msg_mission_set_get_x(msg);
    mission_set->y = mavlink_msg_mission_set_get_y(msg);
    mission_set->z = mavlink_msg_mission_set_get_z(msg);
    mission_set->yaw = mavlink_msg_mission_set_get_yaw(msg);
    mission_set->uav_no_16 = mavlink_msg_mission_set_get_uav_no_16(msg);
    mission_set->mission_no = mavlink_msg_mission_set_get_mission_no(msg);
    mission_set->mission_task = mavlink_msg_mission_set_get_mission_task(msg);
    mission_set->flag = mavlink_msg_mission_set_get_flag(msg);
    mission_set->param1 = mavlink_msg_mission_set_get_param1(msg);
    mission_set->param2 = mavlink_msg_mission_set_get_param2(msg);
    mission_set->param3 = mavlink_msg_mission_set_get_param3(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_SET_LEN? msg->len : MAVLINK_MSG_ID_MISSION_SET_LEN;
        memset(mission_set, 0, MAVLINK_MSG_ID_MISSION_SET_LEN);
    memcpy(mission_set, _MAV_PAYLOAD(msg), len);
#endif
}
