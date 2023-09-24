#pragma once
// MESSAGE GLOBAL_GPS_DOUBLE PACKING

#define MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE 9024


typedef struct __mavlink_global_gps_double_t {
 double latitude; /*<   纬度| Latitude.*/
 double longitude; /*<   经度| Longitude.*/
 double altitude; /*<   高度| Altitude.*/
} mavlink_global_gps_double_t;

#define MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN 24
#define MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN 24
#define MAVLINK_MSG_ID_9024_LEN 24
#define MAVLINK_MSG_ID_9024_MIN_LEN 24

#define MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC 126
#define MAVLINK_MSG_ID_9024_CRC 126



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GLOBAL_GPS_DOUBLE { \
    9024, \
    "GLOBAL_GPS_DOUBLE", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_global_gps_double_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_global_gps_double_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_global_gps_double_t, altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GLOBAL_GPS_DOUBLE { \
    "GLOBAL_GPS_DOUBLE", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_global_gps_double_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_global_gps_double_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_global_gps_double_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a global_gps_double message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude   纬度| Latitude.
 * @param longitude   经度| Longitude.
 * @param altitude   高度| Altitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_gps_double_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double latitude, double longitude, double altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN];
    _mav_put_double(buf, 0, latitude);
    _mav_put_double(buf, 8, longitude);
    _mav_put_double(buf, 16, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN);
#else
    mavlink_global_gps_double_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
}

/**
 * @brief Pack a global_gps_double message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude   纬度| Latitude.
 * @param longitude   经度| Longitude.
 * @param altitude   高度| Altitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_gps_double_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double latitude,double longitude,double altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN];
    _mav_put_double(buf, 0, latitude);
    _mav_put_double(buf, 8, longitude);
    _mav_put_double(buf, 16, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN);
#else
    mavlink_global_gps_double_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
}

/**
 * @brief Encode a global_gps_double struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_gps_double C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_gps_double_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_gps_double_t* global_gps_double)
{
    return mavlink_msg_global_gps_double_pack(system_id, component_id, msg, global_gps_double->latitude, global_gps_double->longitude, global_gps_double->altitude);
}

/**
 * @brief Encode a global_gps_double struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_gps_double C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_gps_double_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_gps_double_t* global_gps_double)
{
    return mavlink_msg_global_gps_double_pack_chan(system_id, component_id, chan, msg, global_gps_double->latitude, global_gps_double->longitude, global_gps_double->altitude);
}

/**
 * @brief Send a global_gps_double message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude   纬度| Latitude.
 * @param longitude   经度| Longitude.
 * @param altitude   高度| Altitude.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_gps_double_send(mavlink_channel_t chan, double latitude, double longitude, double altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN];
    _mav_put_double(buf, 0, latitude);
    _mav_put_double(buf, 8, longitude);
    _mav_put_double(buf, 16, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE, buf, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
#else
    mavlink_global_gps_double_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
#endif
}

/**
 * @brief Send a global_gps_double message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_global_gps_double_send_struct(mavlink_channel_t chan, const mavlink_global_gps_double_t* global_gps_double)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_global_gps_double_send(chan, global_gps_double->latitude, global_gps_double->longitude, global_gps_double->altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE, (const char *)global_gps_double, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_gps_double_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double latitude, double longitude, double altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, latitude);
    _mav_put_double(buf, 8, longitude);
    _mav_put_double(buf, 16, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE, buf, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
#else
    mavlink_global_gps_double_t *packet = (mavlink_global_gps_double_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_GPS_DOUBLE UNPACKING


/**
 * @brief Get field latitude from global_gps_double message
 *
 * @return   纬度| Latitude.
 */
static inline double mavlink_msg_global_gps_double_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field longitude from global_gps_double message
 *
 * @return   经度| Longitude.
 */
static inline double mavlink_msg_global_gps_double_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field altitude from global_gps_double message
 *
 * @return   高度| Altitude.
 */
static inline double mavlink_msg_global_gps_double_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Decode a global_gps_double message into a struct
 *
 * @param msg The message to decode
 * @param global_gps_double C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_gps_double_decode(const mavlink_message_t* msg, mavlink_global_gps_double_t* global_gps_double)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    global_gps_double->latitude = mavlink_msg_global_gps_double_get_latitude(msg);
    global_gps_double->longitude = mavlink_msg_global_gps_double_get_longitude(msg);
    global_gps_double->altitude = mavlink_msg_global_gps_double_get_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN? msg->len : MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN;
        memset(global_gps_double, 0, MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE_LEN);
    memcpy(global_gps_double, _MAV_PAYLOAD(msg), len);
#endif
}
