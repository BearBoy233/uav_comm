/// @file       ros_udp_subtopic.cpp  
/// @brief      SUB ROSTopic => UDP Mavlink msg
/// 
///     ROSTopic Sub 
///     UDP Client
///     读取参数 本机ID 往除其外的其他端口发 port
///
/// @author     bearboy
/// @date       2023-09-24 20:36:14  

#include <ros_udp_commnet.h>
            
// MAX number of UAV in the group
int group_uav_num;
            

// UDP port number | Port number definition (/0-GCS /N-UAV_N) 
int udp_default_port;
int this_udp_sub_port;
std::string udp_boardcast_ip;
int udp_out_port;

// UDP socket
struct sockaddr_in client;
socklen_t client_len = sizeof(client);
int fd;
int sock;
const int opt = -1;
int ret;
ssize_t bytes_sent; // UDP The length of the array sent

uint8_t message_udp[4096];  // byte array, store the raw data to be sent

char buf[1024] = {0};       // array that receives UDP data

// udp send buffer
void send_buffer(mavlink_message_t* mmsg);

// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	close(fd);
	exit(0);// end program here
}

//-----------------------------------------------------------------------------------------
//  ros_cb() 
//                                Subscribe the Ros Topic, package and send [UDP broadcast]
//-----------------------------------------------------------------------------------------

/* Adding part
//-----------------------------------------------------------------------------------------
//	YYYY 类 
//-----------------------------------------------------------------------------------------
void cb_sub_YYYY(const cluster_msgs::YYYY::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_YYYY.AAAA1 = rmsg->AAAA1;
    mt_sub_YYYY.AAAA2 = rmsg->AAAA2;

	mavlink_msg_YYYY_encode( my_id, rmsg->compid, &mmsg, &mt_sub_YYYY);
    send_buffer( &mmsg );
}
*/
            
//-----------------------------------------------------------------------------------------
//	global_gps_double 类 
//-----------------------------------------------------------------------------------------
void cb_sub_global_gps_double(const cluster_msgs::global_gps_double::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

    mt_sub_global_gps_double.latitude = rmsg->latitude;
    mt_sub_global_gps_double.longitude = rmsg->longitude;
    mt_sub_global_gps_double.altitude = rmsg->altitude;
    
	mavlink_msg_global_gps_double_encode( my_id, rmsg->compid, &mmsg, &mt_sub_global_gps_double);
    send_buffer( &mmsg );
}

//-----------------------------------------------------------------------------------------
//	heart_beat 类 
//-----------------------------------------------------------------------------------------
void cb_sub_heart_beat(const cluster_msgs::heart_beat::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

    mt_sub_heart_beat.battery_cell_voltage = rmsg->battery_cell_voltage;
    mt_sub_heart_beat.battery_percentage = rmsg->battery_percentage;
    mt_sub_heart_beat.px4_state = rmsg->px4_state;
    mt_sub_heart_beat.px4_mode = rmsg->px4_mode;
    mt_sub_heart_beat.px4_sys_state = rmsg->px4_sys_state;
    mt_sub_heart_beat.ofb_state = rmsg->ofb_state;
    mt_sub_heart_beat.gps_fix_type = rmsg->gps_fix_type;
    mt_sub_heart_beat.gps_satellites_visible = rmsg->gps_satellites_visible;
    
	mavlink_msg_heart_beat_encode( my_id, rmsg->compid, &mmsg, &mt_sub_heart_beat);
    send_buffer( &mmsg );
}

//-----------------------------------------------------------------------------------------
//	uav_loc_pos_enu_yaw 类 
//-----------------------------------------------------------------------------------------
void cb_sub_uav_loc_pos_enu_yaw(const cluster_msgs::uav_loc_pos_enu_yaw::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

    mt_sub_uav_loc_pos_enu_yaw.x = rmsg->x;
    mt_sub_uav_loc_pos_enu_yaw.y = rmsg->y;
    mt_sub_uav_loc_pos_enu_yaw.z = rmsg->z;
    mt_sub_uav_loc_pos_enu_yaw.yaw = rmsg->yaw;
    
	mavlink_msg_uav_loc_pos_enu_yaw_encode( my_id, rmsg->compid, &mmsg, &mt_sub_uav_loc_pos_enu_yaw);
    send_buffer( &mmsg );
}

//-----------------------------------------------------------------------------------------
//	uav_gps_pos 类 
//-----------------------------------------------------------------------------------------
void cb_sub_uav_gps_pos(const cluster_msgs::uav_gps_pos::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

    mt_sub_uav_gps_pos.latitude = rmsg->latitude;
    mt_sub_uav_gps_pos.longitude = rmsg->longitude;
    mt_sub_uav_gps_pos.altitude = rmsg->altitude;
    mt_sub_uav_gps_pos.hdg_yaw = rmsg->hdg_yaw;
    
	mavlink_msg_uav_gps_pos_encode( my_id, rmsg->compid, &mmsg, &mt_sub_uav_gps_pos);
    send_buffer( &mmsg );
}

//----------------------------------------------------------------------------

            
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_udp_subtopic");

    ros::NodeHandle param_nh("~");
    // receive ROS Topic
    ros::NodeHandle nh_sub("/commnet/send");

    param_nh.param<int>("my_id", my_id, 1);
    param_nh.param<int>("group_uav_num", group_uav_num, 3);
    param_nh.param<int>("udp_default_port", udp_default_port, 13200);
    param_nh.param<std::string>("udp_boardcast_ip", udp_boardcast_ip, "192.168.50.255");

    if ( my_id == ID_GCS )  // 100
    {   this_udp_sub_port = udp_default_port;   }
    else
    {   this_udp_sub_port = udp_default_port + my_id;   }

    udp_out_port = udp_default_port;

    std::cout << std::endl;
    std::cout << "load ID : " << my_id << std::endl;
    std::cout << "group_uav_num : " << group_uav_num << std::endl;
    std::cout << "udp_default_port : " << udp_default_port << std::endl;
    std::cout << "udp_boardcast_ip [bind] : " << udp_boardcast_ip << std::endl;
    std::cout << "this_udp_port_sub[bind] : " << this_udp_sub_port << std::endl;
 
    // Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);
    
	//----------------------------------------------------------------------------
	//                                                              ROS pub&sub
	//----------------------------------------------------------------------------
	/* Adding part
	// get XXXX
	sub_XXXX = nh_sub.subscribe<cluster_msgs::XXXX>("XXXX", 5, cb_sub_XXXX);
	*/

    // get| global_gps_double
    sub_global_gps_double = nh_sub.subscribe<cluster_msgs::global_gps_double>("global_gps_double", 5, cb_sub_global_gps_double);           
    // get| heart_beat
    sub_heart_beat = nh_sub.subscribe<cluster_msgs::heart_beat>("heart_beat", 5, cb_sub_heart_beat);           
    // get| uav_loc_pos_enu_yaw
    sub_uav_loc_pos_enu_yaw = nh_sub.subscribe<cluster_msgs::uav_loc_pos_enu_yaw>("uav_loc_pos_enu_yaw", 5, cb_sub_uav_loc_pos_enu_yaw);           
    // get| uav_gps_pos
    sub_uav_gps_pos = nh_sub.subscribe<cluster_msgs::uav_gps_pos>("uav_gps_pos", 5, cb_sub_uav_gps_pos);           
               

    //------------------------------------------------------------------
    // UDP Client, subscribe & send ROS Topic
    memset(&client, 0, sizeof(client));
	client.sin_family = AF_INET;						            // match the first socket parameter
	client.sin_port = htons(udp_out_port);			                // Fill in the port number in the HTONS | target port number
	client.sin_addr.s_addr = inet_addr(udp_boardcast_ip.c_str());	// IP of the Receiver | 255 for broadcast

    // Create a server socket | int Fd = socket(AF_INET, SOCK_STREAM, 0);
    sock = -1;
    if ( (sock = socket(AF_INET,SOCK_DGRAM,0)) == -1 )
    {   std::cout << "sock error" << std::endl;
        return -1;
    }
    // Set socket type, set port reuse, to prevent restarting the program can not bind the port
    ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&opt, sizeof(opt));
    if( ret == -1 )
    {
        std::cout << "set socket error...\n" << std::endl;
        return -1;
    }
    std::cout << "UDP to " << udp_boardcast_ip << std::endl;
    //------------------------------------------------------------------

    while(ros::ok()) 
    {
        ros::spin(); 
    }

    close(fd);
    return 0;
}


void send_buffer(mavlink_message_t* mmsg)
{
	unsigned len_message_udp;
	len_message_udp  = mavlink_msg_to_send_buffer((uint8_t*)message_udp, mmsg);

    // Check GCS ID | Send data to GCS first
    if (my_id != ID_GCS) 	// 100)
    {
        // Loop | constantly change port number | send out data ???
        udp_out_port = udp_default_port;
        client.sin_port = htons(udp_out_port);                                                          // Fill in the port number in the HTONS
        bytes_sent = sendto(sock, message_udp, len_message_udp, 0, (sockaddr*)&client, client_len);     // Publish message to the broadcast address

        std::cout << "To port " << udp_out_port << ", send_N= "<< bytes_sent << " | send msg" << std::endl;
    }

    for (int i=1; i<=group_uav_num; i++)
    {
        if ( i==my_id )
        {
            std::cout << "Self Port | JUMP" << std::endl;
        }
        else
        {
            // Loop | constantly change port number | send out data ???
            udp_out_port = udp_default_port + i;
            client.sin_port = htons(udp_out_port);			                                                // Fill in the port number in the HTONS
            bytes_sent = sendto(sock, message_udp, len_message_udp, 0, (sockaddr*)&client, client_len);     // Publish message to the broadcast address

            std::cout << "To port " << udp_out_port << ", send_N= "<< bytes_sent << " | send msg" << std::endl;
        }
    }

    std::cout << std::endl;

}
