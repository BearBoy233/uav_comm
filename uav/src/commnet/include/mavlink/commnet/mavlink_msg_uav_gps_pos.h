#pragma once
// MESSAGE UAV_GPS_POS PACKING

#define MAVLINK_MSG_ID_UAV_GPS_POS 6013


typedef struct __mavlink_uav_gps_pos_t {
 int32_t latitude; /*<   (degE7)| 纬度| Latitude.*/
 int32_t longitude; /*<   (degE7)| 经度| Longitude.*/
 float altitude; /*<   (m)| 高度-信息源待定| Altitude.*/
 uint16_t hdg_yaw; /*<   (cdeg)| 偏航角-信息源待定| Vehicle heading(yaw angle), 0.0..359.99 degrees.*/
} mavlink_uav_gps_pos_t;

#define MAVLINK_MSG_ID_UAV_GPS_POS_LEN 14
#define MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN 14
#define MAVLINK_MSG_ID_6013_LEN 14
#define MAVLINK_MSG_ID_6013_MIN_LEN 14

#define MAVLINK_MSG_ID_UAV_GPS_POS_CRC 38
#define MAVLINK_MSG_ID_6013_CRC 38



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_GPS_POS { \
    6013, \
    "UAV_GPS_POS", \
    4, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uav_gps_pos_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uav_gps_pos_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_gps_pos_t, altitude) }, \
         { "hdg_yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_uav_gps_pos_t, hdg_yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_GPS_POS { \
    "UAV_GPS_POS", \
    4, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uav_gps_pos_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uav_gps_pos_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_gps_pos_t, altitude) }, \
         { "hdg_yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_uav_gps_pos_t, hdg_yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_gps_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude   (degE7)| 纬度| Latitude.
 * @param longitude   (degE7)| 经度| Longitude.
 * @param altitude   (m)| 高度-信息源待定| Altitude.
 * @param hdg_yaw   (cdeg)| 偏航角-信息源待定| Vehicle heading(yaw angle), 0.0..359.99 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_gps_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, float altitude, uint16_t hdg_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_GPS_POS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, hdg_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_GPS_POS_LEN);
#else
    mavlink_uav_gps_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.hdg_yaw = hdg_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_GPS_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_GPS_POS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
}

/**
 * @brief Pack a uav_gps_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude   (degE7)| 纬度| Latitude.
 * @param longitude   (degE7)| 经度| Longitude.
 * @param altitude   (m)| 高度-信息源待定| Altitude.
 * @param hdg_yaw   (cdeg)| 偏航角-信息源待定| Vehicle heading(yaw angle), 0.0..359.99 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_gps_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,float altitude,uint16_t hdg_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_GPS_POS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, hdg_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_GPS_POS_LEN);
#else
    mavlink_uav_gps_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.hdg_yaw = hdg_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_GPS_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_GPS_POS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
}

/**
 * @brief Encode a uav_gps_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_gps_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_gps_pos_t* uav_gps_pos)
{
    return mavlink_msg_uav_gps_pos_pack(system_id, component_id, msg, uav_gps_pos->latitude, uav_gps_pos->longitude, uav_gps_pos->altitude, uav_gps_pos->hdg_yaw);
}

/**
 * @brief Encode a uav_gps_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_gps_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_gps_pos_t* uav_gps_pos)
{
    return mavlink_msg_uav_gps_pos_pack_chan(system_id, component_id, chan, msg, uav_gps_pos->latitude, uav_gps_pos->longitude, uav_gps_pos->altitude, uav_gps_pos->hdg_yaw);
}

/**
 * @brief Send a uav_gps_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude   (degE7)| 纬度| Latitude.
 * @param longitude   (degE7)| 经度| Longitude.
 * @param altitude   (m)| 高度-信息源待定| Altitude.
 * @param hdg_yaw   (cdeg)| 偏航角-信息源待定| Vehicle heading(yaw angle), 0.0..359.99 degrees.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_gps_pos_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, float altitude, uint16_t hdg_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_GPS_POS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, hdg_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_GPS_POS, buf, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
#else
    mavlink_uav_gps_pos_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.hdg_yaw = hdg_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_GPS_POS, (const char *)&packet, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
#endif
}

/**
 * @brief Send a uav_gps_pos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_gps_pos_send_struct(mavlink_channel_t chan, const mavlink_uav_gps_pos_t* uav_gps_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_gps_pos_send(chan, uav_gps_pos->latitude, uav_gps_pos->longitude, uav_gps_pos->altitude, uav_gps_pos->hdg_yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_GPS_POS, (const char *)uav_gps_pos, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_GPS_POS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_gps_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, float altitude, uint16_t hdg_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, hdg_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_GPS_POS, buf, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
#else
    mavlink_uav_gps_pos_t *packet = (mavlink_uav_gps_pos_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->hdg_yaw = hdg_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_GPS_POS, (const char *)packet, MAVLINK_MSG_ID_UAV_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_LEN, MAVLINK_MSG_ID_UAV_GPS_POS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_GPS_POS UNPACKING


/**
 * @brief Get field latitude from uav_gps_pos message
 *
 * @return   (degE7)| 纬度| Latitude.
 */
static inline int32_t mavlink_msg_uav_gps_pos_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from uav_gps_pos message
 *
 * @return   (degE7)| 经度| Longitude.
 */
static inline int32_t mavlink_msg_uav_gps_pos_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from uav_gps_pos message
 *
 * @return   (m)| 高度-信息源待定| Altitude.
 */
static inline float mavlink_msg_uav_gps_pos_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hdg_yaw from uav_gps_pos message
 *
 * @return   (cdeg)| 偏航角-信息源待定| Vehicle heading(yaw angle), 0.0..359.99 degrees.
 */
static inline uint16_t mavlink_msg_uav_gps_pos_get_hdg_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a uav_gps_pos message into a struct
 *
 * @param msg The message to decode
 * @param uav_gps_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_gps_pos_decode(const mavlink_message_t* msg, mavlink_uav_gps_pos_t* uav_gps_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_gps_pos->latitude = mavlink_msg_uav_gps_pos_get_latitude(msg);
    uav_gps_pos->longitude = mavlink_msg_uav_gps_pos_get_longitude(msg);
    uav_gps_pos->altitude = mavlink_msg_uav_gps_pos_get_altitude(msg);
    uav_gps_pos->hdg_yaw = mavlink_msg_uav_gps_pos_get_hdg_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_GPS_POS_LEN? msg->len : MAVLINK_MSG_ID_UAV_GPS_POS_LEN;
        memset(uav_gps_pos, 0, MAVLINK_MSG_ID_UAV_GPS_POS_LEN);
    memcpy(uav_gps_pos, _MAV_PAYLOAD(msg), len);
#endif
}
