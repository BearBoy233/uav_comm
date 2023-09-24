#pragma once
// MESSAGE HEART_BEAT PACKING

#define MAVLINK_MSG_ID_HEART_BEAT 6001


typedef struct __mavlink_heart_beat_t {
 float battery_cell_voltage; /*<   Subscribe ROSTopic /battery/#cell_voltage[0]| 单节电池的电压(V)| Battery cell voltage.*/
 float battery_percentage; /*<   Subscribe ROSTopic /battery/#percentage| 电量百分比(%)| Battery percentage.*/
 uint8_t px4_state; /*<   Subscribe ROSTopic /mavros/state| 当前PX4飞控的状态| From high to low, the flag bits are [7-connected, 6-armed, 5-guided, 4-manual_input, 3-battery_on], 2-0 ToBeUsedLater.*/
 uint8_t px4_mode; /*<   Subscribe ROSTopic /mavros/state/#mode| 当前PX4飞控的模式| Current autopilot mode.*/
 uint8_t px4_sys_state; /*<   Subscribe ROSTopic /mavros/state/#system_state| 当前PX4飞控的系统状态| Current system state.*/
 uint8_t ofb_state; /*<   Subscribe ROSTopic /this/state/onboardflight| 当前机载计算机的控制主程序的状态【待定】| (ENUM STATE_ONBOARD_FLIGHT).*/
 uint8_t gps_fix_type; /*<   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_fix_type| GPS定位类型.*/
 uint8_t gps_satellites_visible; /*<   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible| GPS可见星数.*/
} mavlink_heart_beat_t;

#define MAVLINK_MSG_ID_HEART_BEAT_LEN 14
#define MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN 14
#define MAVLINK_MSG_ID_6001_LEN 14
#define MAVLINK_MSG_ID_6001_MIN_LEN 14

#define MAVLINK_MSG_ID_HEART_BEAT_CRC 137
#define MAVLINK_MSG_ID_6001_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HEART_BEAT { \
    6001, \
    "HEART_BEAT", \
    8, \
    {  { "px4_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heart_beat_t, px4_state) }, \
         { "px4_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heart_beat_t, px4_mode) }, \
         { "px4_sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heart_beat_t, px4_sys_state) }, \
         { "ofb_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heart_beat_t, ofb_state) }, \
         { "battery_cell_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_heart_beat_t, battery_cell_voltage) }, \
         { "battery_percentage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_heart_beat_t, battery_percentage) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heart_beat_t, gps_fix_type) }, \
         { "gps_satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heart_beat_t, gps_satellites_visible) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HEART_BEAT { \
    "HEART_BEAT", \
    8, \
    {  { "px4_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heart_beat_t, px4_state) }, \
         { "px4_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heart_beat_t, px4_mode) }, \
         { "px4_sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heart_beat_t, px4_sys_state) }, \
         { "ofb_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heart_beat_t, ofb_state) }, \
         { "battery_cell_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_heart_beat_t, battery_cell_voltage) }, \
         { "battery_percentage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_heart_beat_t, battery_percentage) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heart_beat_t, gps_fix_type) }, \
         { "gps_satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heart_beat_t, gps_satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a heart_beat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param px4_state   Subscribe ROSTopic /mavros/state| 当前PX4飞控的状态| From high to low, the flag bits are [7-connected, 6-armed, 5-guided, 4-manual_input, 3-battery_on], 2-0 ToBeUsedLater.
 * @param px4_mode   Subscribe ROSTopic /mavros/state/#mode| 当前PX4飞控的模式| Current autopilot mode.
 * @param px4_sys_state   Subscribe ROSTopic /mavros/state/#system_state| 当前PX4飞控的系统状态| Current system state.
 * @param ofb_state   Subscribe ROSTopic /this/state/onboardflight| 当前机载计算机的控制主程序的状态【待定】| (ENUM STATE_ONBOARD_FLIGHT).
 * @param battery_cell_voltage   Subscribe ROSTopic /battery/#cell_voltage[0]| 单节电池的电压(V)| Battery cell voltage.
 * @param battery_percentage   Subscribe ROSTopic /battery/#percentage| 电量百分比(%)| Battery percentage.
 * @param gps_fix_type   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_fix_type| GPS定位类型.
 * @param gps_satellites_visible   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible| GPS可见星数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heart_beat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t px4_state, uint8_t px4_mode, uint8_t px4_sys_state, uint8_t ofb_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEART_BEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_state);
    _mav_put_uint8_t(buf, 9, px4_mode);
    _mav_put_uint8_t(buf, 10, px4_sys_state);
    _mav_put_uint8_t(buf, 11, ofb_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEART_BEAT_LEN);
#else
    mavlink_heart_beat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_state = px4_state;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.ofb_state = ofb_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEART_BEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEART_BEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
}

/**
 * @brief Pack a heart_beat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_state   Subscribe ROSTopic /mavros/state| 当前PX4飞控的状态| From high to low, the flag bits are [7-connected, 6-armed, 5-guided, 4-manual_input, 3-battery_on], 2-0 ToBeUsedLater.
 * @param px4_mode   Subscribe ROSTopic /mavros/state/#mode| 当前PX4飞控的模式| Current autopilot mode.
 * @param px4_sys_state   Subscribe ROSTopic /mavros/state/#system_state| 当前PX4飞控的系统状态| Current system state.
 * @param ofb_state   Subscribe ROSTopic /this/state/onboardflight| 当前机载计算机的控制主程序的状态【待定】| (ENUM STATE_ONBOARD_FLIGHT).
 * @param battery_cell_voltage   Subscribe ROSTopic /battery/#cell_voltage[0]| 单节电池的电压(V)| Battery cell voltage.
 * @param battery_percentage   Subscribe ROSTopic /battery/#percentage| 电量百分比(%)| Battery percentage.
 * @param gps_fix_type   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_fix_type| GPS定位类型.
 * @param gps_satellites_visible   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible| GPS可见星数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heart_beat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t px4_state,uint8_t px4_mode,uint8_t px4_sys_state,uint8_t ofb_state,float battery_cell_voltage,float battery_percentage,uint8_t gps_fix_type,uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEART_BEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_state);
    _mav_put_uint8_t(buf, 9, px4_mode);
    _mav_put_uint8_t(buf, 10, px4_sys_state);
    _mav_put_uint8_t(buf, 11, ofb_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEART_BEAT_LEN);
#else
    mavlink_heart_beat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_state = px4_state;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.ofb_state = ofb_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEART_BEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEART_BEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
}

/**
 * @brief Encode a heart_beat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heart_beat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heart_beat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heart_beat_t* heart_beat)
{
    return mavlink_msg_heart_beat_pack(system_id, component_id, msg, heart_beat->px4_state, heart_beat->px4_mode, heart_beat->px4_sys_state, heart_beat->ofb_state, heart_beat->battery_cell_voltage, heart_beat->battery_percentage, heart_beat->gps_fix_type, heart_beat->gps_satellites_visible);
}

/**
 * @brief Encode a heart_beat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heart_beat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heart_beat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heart_beat_t* heart_beat)
{
    return mavlink_msg_heart_beat_pack_chan(system_id, component_id, chan, msg, heart_beat->px4_state, heart_beat->px4_mode, heart_beat->px4_sys_state, heart_beat->ofb_state, heart_beat->battery_cell_voltage, heart_beat->battery_percentage, heart_beat->gps_fix_type, heart_beat->gps_satellites_visible);
}

/**
 * @brief Send a heart_beat message
 * @param chan MAVLink channel to send the message
 *
 * @param px4_state   Subscribe ROSTopic /mavros/state| 当前PX4飞控的状态| From high to low, the flag bits are [7-connected, 6-armed, 5-guided, 4-manual_input, 3-battery_on], 2-0 ToBeUsedLater.
 * @param px4_mode   Subscribe ROSTopic /mavros/state/#mode| 当前PX4飞控的模式| Current autopilot mode.
 * @param px4_sys_state   Subscribe ROSTopic /mavros/state/#system_state| 当前PX4飞控的系统状态| Current system state.
 * @param ofb_state   Subscribe ROSTopic /this/state/onboardflight| 当前机载计算机的控制主程序的状态【待定】| (ENUM STATE_ONBOARD_FLIGHT).
 * @param battery_cell_voltage   Subscribe ROSTopic /battery/#cell_voltage[0]| 单节电池的电压(V)| Battery cell voltage.
 * @param battery_percentage   Subscribe ROSTopic /battery/#percentage| 电量百分比(%)| Battery percentage.
 * @param gps_fix_type   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_fix_type| GPS定位类型.
 * @param gps_satellites_visible   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible| GPS可见星数.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heart_beat_send(mavlink_channel_t chan, uint8_t px4_state, uint8_t px4_mode, uint8_t px4_sys_state, uint8_t ofb_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEART_BEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_state);
    _mav_put_uint8_t(buf, 9, px4_mode);
    _mav_put_uint8_t(buf, 10, px4_sys_state);
    _mav_put_uint8_t(buf, 11, ofb_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEART_BEAT, buf, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
#else
    mavlink_heart_beat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_state = px4_state;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.ofb_state = ofb_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEART_BEAT, (const char *)&packet, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
#endif
}

/**
 * @brief Send a heart_beat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_heart_beat_send_struct(mavlink_channel_t chan, const mavlink_heart_beat_t* heart_beat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_heart_beat_send(chan, heart_beat->px4_state, heart_beat->px4_mode, heart_beat->px4_sys_state, heart_beat->ofb_state, heart_beat->battery_cell_voltage, heart_beat->battery_percentage, heart_beat->gps_fix_type, heart_beat->gps_satellites_visible);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEART_BEAT, (const char *)heart_beat, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_HEART_BEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heart_beat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t px4_state, uint8_t px4_mode, uint8_t px4_sys_state, uint8_t ofb_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_state);
    _mav_put_uint8_t(buf, 9, px4_mode);
    _mav_put_uint8_t(buf, 10, px4_sys_state);
    _mav_put_uint8_t(buf, 11, ofb_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEART_BEAT, buf, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
#else
    mavlink_heart_beat_t *packet = (mavlink_heart_beat_t *)msgbuf;
    packet->battery_cell_voltage = battery_cell_voltage;
    packet->battery_percentage = battery_percentage;
    packet->px4_state = px4_state;
    packet->px4_mode = px4_mode;
    packet->px4_sys_state = px4_sys_state;
    packet->ofb_state = ofb_state;
    packet->gps_fix_type = gps_fix_type;
    packet->gps_satellites_visible = gps_satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEART_BEAT, (const char *)packet, MAVLINK_MSG_ID_HEART_BEAT_MIN_LEN, MAVLINK_MSG_ID_HEART_BEAT_LEN, MAVLINK_MSG_ID_HEART_BEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE HEART_BEAT UNPACKING


/**
 * @brief Get field px4_state from heart_beat message
 *
 * @return   Subscribe ROSTopic /mavros/state| 当前PX4飞控的状态| From high to low, the flag bits are [7-connected, 6-armed, 5-guided, 4-manual_input, 3-battery_on], 2-0 ToBeUsedLater.
 */
static inline uint8_t mavlink_msg_heart_beat_get_px4_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field px4_mode from heart_beat message
 *
 * @return   Subscribe ROSTopic /mavros/state/#mode| 当前PX4飞控的模式| Current autopilot mode.
 */
static inline uint8_t mavlink_msg_heart_beat_get_px4_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field px4_sys_state from heart_beat message
 *
 * @return   Subscribe ROSTopic /mavros/state/#system_state| 当前PX4飞控的系统状态| Current system state.
 */
static inline uint8_t mavlink_msg_heart_beat_get_px4_sys_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field ofb_state from heart_beat message
 *
 * @return   Subscribe ROSTopic /this/state/onboardflight| 当前机载计算机的控制主程序的状态【待定】| (ENUM STATE_ONBOARD_FLIGHT).
 */
static inline uint8_t mavlink_msg_heart_beat_get_ofb_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field battery_cell_voltage from heart_beat message
 *
 * @return   Subscribe ROSTopic /battery/#cell_voltage[0]| 单节电池的电压(V)| Battery cell voltage.
 */
static inline float mavlink_msg_heart_beat_get_battery_cell_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field battery_percentage from heart_beat message
 *
 * @return   Subscribe ROSTopic /battery/#percentage| 电量百分比(%)| Battery percentage.
 */
static inline float mavlink_msg_heart_beat_get_battery_percentage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field gps_fix_type from heart_beat message
 *
 * @return   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_fix_type| GPS定位类型.
 */
static inline uint8_t mavlink_msg_heart_beat_get_gps_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field gps_satellites_visible from heart_beat message
 *
 * @return   Subscribe ROSTopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible| GPS可见星数.
 */
static inline uint8_t mavlink_msg_heart_beat_get_gps_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a heart_beat message into a struct
 *
 * @param msg The message to decode
 * @param heart_beat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heart_beat_decode(const mavlink_message_t* msg, mavlink_heart_beat_t* heart_beat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    heart_beat->battery_cell_voltage = mavlink_msg_heart_beat_get_battery_cell_voltage(msg);
    heart_beat->battery_percentage = mavlink_msg_heart_beat_get_battery_percentage(msg);
    heart_beat->px4_state = mavlink_msg_heart_beat_get_px4_state(msg);
    heart_beat->px4_mode = mavlink_msg_heart_beat_get_px4_mode(msg);
    heart_beat->px4_sys_state = mavlink_msg_heart_beat_get_px4_sys_state(msg);
    heart_beat->ofb_state = mavlink_msg_heart_beat_get_ofb_state(msg);
    heart_beat->gps_fix_type = mavlink_msg_heart_beat_get_gps_fix_type(msg);
    heart_beat->gps_satellites_visible = mavlink_msg_heart_beat_get_gps_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HEART_BEAT_LEN? msg->len : MAVLINK_MSG_ID_HEART_BEAT_LEN;
        memset(heart_beat, 0, MAVLINK_MSG_ID_HEART_BEAT_LEN);
    memcpy(heart_beat, _MAV_PAYLOAD(msg), len);
#endif
}
