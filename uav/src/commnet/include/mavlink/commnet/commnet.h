/** @file
 *  @brief MAVLink comm protocol generated from commnet.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_COMMNET_H
#define MAVLINK_COMMNET_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_COMMNET.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{6001, 137, 14, 14, 0, 0, 0}, {6012, 79, 16, 16, 0, 0, 0}, {6013, 38, 14, 14, 0, 0, 0}, {9024, 126, 24, 24, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_COMMNET

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_global_gps_double.h"
#include "./mavlink_msg_heart_beat.h"
#include "./mavlink_msg_uav_loc_pos_enu_yaw.h"
#include "./mavlink_msg_uav_gps_pos.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEART_BEAT, MAVLINK_MESSAGE_INFO_UAV_LOC_POS_ENU_YAW, MAVLINK_MESSAGE_INFO_UAV_GPS_POS, MAVLINK_MESSAGE_INFO_GLOBAL_GPS_DOUBLE}
# define MAVLINK_MESSAGE_NAMES {{ "GLOBAL_GPS_DOUBLE", 9024 }, { "HEART_BEAT", 6001 }, { "UAV_GPS_POS", 6013 }, { "UAV_LOC_POS_ENU_YAW", 6012 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_COMMNET_H
