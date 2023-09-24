/// @file       ros_udp_commnet.h  
/// @brief      Pack and unpack ROSTopic
///
///     Implement the conversion between custom Mavlink Data and custom ROS MSG messages. 
///     [commnet] <==> [cluster_msgs]
///     Support serial port and UDP communication
///
/// @author     bearboy
/// @date       2023-09-24 20:36:14


// The header file contains the generic parameter definitions for the cluster system.
#include <cluster_msgs/common_define.h>

#include <ros/ros.h> 

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h> 
//#include <fcntl.h>

// Custom Mavlink header file
#include <mavlink/commnet/mavlink.h>	

// Custom ROS msg header file
/* adding structure
#include <cluster_msgs/AABBCC.h> 
*/
#include <cluster_msgs/global_gps_double.h>
#include <cluster_msgs/heart_beat.h>
#include <cluster_msgs/uav_loc_pos_enu_yaw.h>
#include <cluster_msgs/uav_gps_pos.h>

//-------------------------------------------------------------------------------
// Test Print flag
// int Flag_1ShowRn;	// 标志位 1将读取的 Rn 显示在屏幕上

// system param
int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
int companion_id;	// compid 接收端编号	->发送时  接收端编号	/ 接收时 接收端编号
int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

// ros::Rate *loop_rate;


                        
//-------------------------------------------------------------------------------
// help func
void quit_handler(int sig);	// QuitSignalHandler  Called when you press Ctrl-C



//-------------------------------------------------------------------------------
// sub pub func       
//-------------------------------------------------------
/* adding structure
// AABBCC
// pub
ros::Publisher      pub_AABBCC;
mavlink_AABBCC_t        mt_pub_AABBCC;
cluster_msgs::AABBCC        msg_pub_AABBCC;
// sub
ros::Subscriber     sub_AABBCC;
mavlink_AABBCC_t        mt_sub_AABBCC;
cluster_msgs::AABBCC        msg_sub_AABBCC;
*/

// global_gps_double
// pub
ros::Publisher      pub_global_gps_double;
mavlink_global_gps_double_t     mt_pub_global_gps_double;
cluster_msgs::global_gps_double     msg_pub_global_gps_double;
// sub
ros::Subscriber     sub_global_gps_double;
mavlink_global_gps_double_t     mt_sub_global_gps_double;
cluster_msgs::global_gps_double     msg_sub_global_gps_double;
            
// heart_beat
// pub
ros::Publisher      pub_heart_beat;
mavlink_heart_beat_t     mt_pub_heart_beat;
cluster_msgs::heart_beat     msg_pub_heart_beat;
// sub
ros::Subscriber     sub_heart_beat;
mavlink_heart_beat_t     mt_sub_heart_beat;
cluster_msgs::heart_beat     msg_sub_heart_beat;
            
// uav_loc_pos_enu_yaw
// pub
ros::Publisher      pub_uav_loc_pos_enu_yaw;
mavlink_uav_loc_pos_enu_yaw_t     mt_pub_uav_loc_pos_enu_yaw;
cluster_msgs::uav_loc_pos_enu_yaw     msg_pub_uav_loc_pos_enu_yaw;
// sub
ros::Subscriber     sub_uav_loc_pos_enu_yaw;
mavlink_uav_loc_pos_enu_yaw_t     mt_sub_uav_loc_pos_enu_yaw;
cluster_msgs::uav_loc_pos_enu_yaw     msg_sub_uav_loc_pos_enu_yaw;
            
// uav_gps_pos
// pub
ros::Publisher      pub_uav_gps_pos;
mavlink_uav_gps_pos_t     mt_pub_uav_gps_pos;
cluster_msgs::uav_gps_pos     msg_pub_uav_gps_pos;
// sub
ros::Subscriber     sub_uav_gps_pos;
mavlink_uav_gps_pos_t     mt_sub_uav_gps_pos;
cluster_msgs::uav_gps_pos     msg_sub_uav_gps_pos;
            


