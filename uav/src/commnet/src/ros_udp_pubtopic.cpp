/// @file       ros_udp_pubtopic.cpp  
/// @brief      UDP Mavlink msg => PUB ROSTopic
/// 
///     ROSTopic Pub 
///     UDP Server
///     读取参数 本机ID 仅订阅对应的端口 port
///
/// @author     bearboy
/// @date       2023-09-24 20:36:14  

#include <ros_udp_commnet.h>

// Test output
int Flag_1ShowRn = 2;

// Mavlink data processing
void handle_recieve_msg();
int read_buffer(mavlink_message_t &message, uint8_t cp);
void publish_commnet_recieve(mavlink_message_t mmsg);

char buf[1024] = {0};       // An array that receives UDP data
int recvlen;                // The length of the receive array

uint8_t Rbuffer_rx[4096];       // byte array, store the raw data received by the serial port 
//size_t  Rn;                   // The length of the data received by the byte array
//size_t last_Rn = 0;           // Used when the read is incomplete

// socket
int fd;
// UDP port number | Port number definition (/0-GCS /N-UAV_N) 
int udp_default_port;
int this_udp_sub_port;

// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	close(fd);
	exit(0);// end program here
}

            
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_udp_sub");
    //Create a handle
    // (although this handle is not used later, the runtime process will fail if it is not created) 

    ros::NodeHandle param_nh("~");
    ros::NodeHandle nh_pub("/commnet/receive");

    param_nh.param<int>("my_id", my_id, ID_GCS);    //100
    param_nh.param<int>("udp_default_port", udp_default_port, 13200);

    if ( my_id == ID_GCS )  //100
    {   this_udp_sub_port = udp_default_port;   
    }
    else
    {   this_udp_sub_port = udp_default_port + my_id;   
    }

    std::cout << std::endl;
    std::cout << "load ID : " << my_id << std::endl;
    std::cout << "udp_default_port : " << udp_default_port << std::endl;
    std::cout << "this_udp_port_sub[bind] : " << this_udp_sub_port << std::endl;

    // Responds to early exits signaled with Ctrl-C. 
    signal(SIGINT, quit_handler);
            
    // ROS publish
    /* Adding part
    // Get&Pub UDP msg| XXXX 
    pub_XXXX = nh_pub.advertise<cluster_msgs::XXXX>("XXXX", 5);
    */
    // Get&Pub UDP msg| global_gps_double
    pub_global_gps_double = nh_pub.advertise<cluster_msgs::global_gps_double>("global_gps_double", 5);            
    // Get&Pub UDP msg| heart_beat
    pub_heart_beat = nh_pub.advertise<cluster_msgs::heart_beat>("heart_beat", 5);            
    // Get&Pub UDP msg| uav_loc_pos_enu_yaw
    pub_uav_loc_pos_enu_yaw = nh_pub.advertise<cluster_msgs::uav_loc_pos_enu_yaw>("uav_loc_pos_enu_yaw", 5);            
    // Get&Pub UDP msg| uav_gps_pos
    pub_uav_gps_pos = nh_pub.advertise<cluster_msgs::uav_gps_pos>("uav_gps_pos", 5);            
    
            
    //------------------------------------------------------------------
    // [UDP Server] for binding to receive port [broadcast/multicast?]
    // Server constructor
    struct sockaddr_in server;
    socklen_t server_len = sizeof(server);
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;                    //Matches the first socket parameter
    server.sin_port = htons(this_udp_sub_port);     //Set the server port number
    server.sin_addr.s_addr = htonl(INADDR_ANY);     //Set to any network card on the current computer | from any IP

    // bind socket to local_port
    // Create the socket server socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if ( fd == -1 )
    {   std::cout << "error | socket failed" << std::endl;
        close(fd);  exit(1);
    }

    if ( bind(fd, (struct sockaddr*)&server, sizeof(server)) == -1)
    {   std::cout << "error | bind failed" << std::endl;
        close(fd);  exit(1);
    }

    std::cout << "Set udp service [ip|port] : " << inet_ntoa(server.sin_addr) << ":" << this_udp_sub_port << std::endl;
    //------------------------------------------------------------------

    while(ros::ok()) 
    {
        // [blocking] receives data to save to BUF and returns the data length.
        recvlen = recvfrom(fd, Rbuffer_rx, sizeof(Rbuffer_rx), 0, (struct sockaddr*)&server, &server_len);

        if (recvlen == -1)
        {   std::cout << "error | recvform error" << std::endl; 
            close(fd);  exit(1);    
        }
        
        // UDP | If there is data to be read, recvlen>0
		if (recvlen != 0) 
		{
			if (Flag_1ShowRn) 
			{   // print out
				std::cout << "[No." << my_id << "] Receive " << std::dec << (int)recvlen <<" byte." << std::endl;
				if (Flag_1ShowRn == 2) 
				{   for (int i = 0; i < recvlen; i++) 
					{   std::cout << std::hex << (Rbuffer_rx[i] & 0xff) << " "; 
                    }
					std::cout << std::endl;
				}
			}
            handle_recieve_msg();
		}   
    }
    
    close(fd);
    return 0;
}

void handle_recieve_msg()
{
    bool success;   // Flag bit, mavlink data is resolved correctly

	// without additional frame headers, resolve directly in the mavlink function
	// parse Message by byte
	for (int i = 0; i < recvlen; i++) 
	{	
		mavlink_message_t mmsg;		// The message packet obtained after unpacking
		// const mavlink_message_t *mmsg;
		success = read_buffer(mmsg, Rbuffer_rx[i] & 0xff);
		if (success) 
		{ 
			if (Flag_1ShowRn) 
			{
			    std::cout << "[No." << my_id << "] Receive a complete Mavlink message. No = "  << mmsg.msgid << std::endl;	
			}

			// The message is received by ALL UAVS/ THIS UAV/ GCS
			if ( (mmsg.compid == ID_ALL) || (mmsg.compid == my_id) ) // || (my_id == ID_GCS) 
			{						
		        // The message type is resolved here and publish
                publish_commnet_recieve(mmsg);  // read and publish as Rostopic
		    }
		}
	}
							
	if (!success) 
	{	
		//std::cout<< "Not a complete" << endl;
	}
}

// The data is checked by bytes
int read_buffer(mavlink_message_t &message, uint8_t cp)
{
	mavlink_status_t status;
	uint8_t msgReceived = false;

	// PARSE MESSAGE
	msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);    // MAVLINK_COMM_1 can be changed to the number of channels

	return msgReceived;
}

//-----------------------------------------------------------------------------------------
// The received mavlink data is converted to ROSMSG and published        
//                                          publish_commnet_recieve(mavlink_message_t mmsg)
//-----------------------------------------------------------------------------------------
void publish_commnet_recieve(mavlink_message_t mmsg)
{
	if (Flag_1ShowRn) 
	{
		std::cout << "[No." << my_id << "] ROSPub MAVLink MSG " << mmsg.msgid << std::endl;
	}

    switch (mmsg.msgid) 
	{
		/* Adding part
        //-----------------------------------------------------------------------------------------
		//	AABBCC 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_AABBCC:
			// 解码
			mavlink_msg_AABBCC_decode(&mmsg, &(mt_pub_AABBCC));
			// 赋值_common
			msg_pub_AABBCC.sysid = mmsg.sysid;
			msg_pub_AABBCC.compid = mmsg.compid;
			// 赋值_main
			msg_pub_AABBCC.XXXXXX1 = mt_pub_AABBCC.XXXXXX1;
			msg_pub_AABBCC.XXXXXX2 = mt_pub_AABBCC.XXXXXX2;
			// 发布
			pub_AABBCC.publish(msg_pub_AABBCC);
			// rmsg pub_AABBCC.
		break;
		*/
            
        
        //-----------------------------------------------------------------------------------------
		//	global_gps_double 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_GLOBAL_GPS_DOUBLE:
            // 解码
			mavlink_msg_global_gps_double_decode(&mmsg, &(mt_pub_global_gps_double));
			// 赋值_common
			msg_pub_global_gps_double.sysid = mmsg.sysid;
			msg_pub_global_gps_double.compid = mmsg.compid;
			// 赋值_main
            msg_pub_global_gps_double.latitude = mt_pub_global_gps_double.latitude;
            msg_pub_global_gps_double.longitude = mt_pub_global_gps_double.longitude;
            msg_pub_global_gps_double.altitude = mt_pub_global_gps_double.altitude;
            // 发布
			pub_global_gps_double.publish(msg_pub_global_gps_double);
			// rmsg pub_global_gps_double.
		break;
        
        //-----------------------------------------------------------------------------------------
		//	heart_beat 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_HEART_BEAT:
            // 解码
			mavlink_msg_heart_beat_decode(&mmsg, &(mt_pub_heart_beat));
			// 赋值_common
			msg_pub_heart_beat.sysid = mmsg.sysid;
			msg_pub_heart_beat.compid = mmsg.compid;
			// 赋值_main
            msg_pub_heart_beat.battery_cell_voltage = mt_pub_heart_beat.battery_cell_voltage;
            msg_pub_heart_beat.battery_percentage = mt_pub_heart_beat.battery_percentage;
            msg_pub_heart_beat.px4_state = mt_pub_heart_beat.px4_state;
            msg_pub_heart_beat.px4_mode = mt_pub_heart_beat.px4_mode;
            msg_pub_heart_beat.px4_sys_state = mt_pub_heart_beat.px4_sys_state;
            msg_pub_heart_beat.ofb_state = mt_pub_heart_beat.ofb_state;
            msg_pub_heart_beat.gps_fix_type = mt_pub_heart_beat.gps_fix_type;
            msg_pub_heart_beat.gps_satellites_visible = mt_pub_heart_beat.gps_satellites_visible;
            // 发布
			pub_heart_beat.publish(msg_pub_heart_beat);
			// rmsg pub_heart_beat.
		break;
        
        //-----------------------------------------------------------------------------------------
		//	uav_loc_pos_enu_yaw 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_UAV_LOC_POS_ENU_YAW:
            // 解码
			mavlink_msg_uav_loc_pos_enu_yaw_decode(&mmsg, &(mt_pub_uav_loc_pos_enu_yaw));
			// 赋值_common
			msg_pub_uav_loc_pos_enu_yaw.sysid = mmsg.sysid;
			msg_pub_uav_loc_pos_enu_yaw.compid = mmsg.compid;
			// 赋值_main
            msg_pub_uav_loc_pos_enu_yaw.x = mt_pub_uav_loc_pos_enu_yaw.x;
            msg_pub_uav_loc_pos_enu_yaw.y = mt_pub_uav_loc_pos_enu_yaw.y;
            msg_pub_uav_loc_pos_enu_yaw.z = mt_pub_uav_loc_pos_enu_yaw.z;
            msg_pub_uav_loc_pos_enu_yaw.yaw = mt_pub_uav_loc_pos_enu_yaw.yaw;
            // 发布
			pub_uav_loc_pos_enu_yaw.publish(msg_pub_uav_loc_pos_enu_yaw);
			// rmsg pub_uav_loc_pos_enu_yaw.
		break;
        
        //-----------------------------------------------------------------------------------------
		//	uav_gps_pos 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_UAV_GPS_POS:
            // 解码
			mavlink_msg_uav_gps_pos_decode(&mmsg, &(mt_pub_uav_gps_pos));
			// 赋值_common
			msg_pub_uav_gps_pos.sysid = mmsg.sysid;
			msg_pub_uav_gps_pos.compid = mmsg.compid;
			// 赋值_main
            msg_pub_uav_gps_pos.latitude = mt_pub_uav_gps_pos.latitude;
            msg_pub_uav_gps_pos.longitude = mt_pub_uav_gps_pos.longitude;
            msg_pub_uav_gps_pos.altitude = mt_pub_uav_gps_pos.altitude;
            msg_pub_uav_gps_pos.hdg_yaw = mt_pub_uav_gps_pos.hdg_yaw;
            // 发布
			pub_uav_gps_pos.publish(msg_pub_uav_gps_pos);
			// rmsg pub_uav_gps_pos.
		break;

        default:

		break;
	}

}
