#!/usr/bin/env python
'''
Generate the header file

'''
from __future__ import print_function
from future.utils import iteritems

from builtins import range
from builtins import object

import os
import mavparse, mavtemplate
from datetime import datetime

t = mavtemplate.MAVTemplate()


# generate ros_udp_commnet.h
def commnet_generate_ros_udp_commnet_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, "ros_udp_commnet.h"), mode='w')
    t.write(f, '''
/// @file       ros_udp_commnet.h  
/// @brief      Pack and unpack ROSTopic
///
///     Implement the conversion between custom Mavlink Data and custom ROS MSG messages. 
///     [commnet] <==> [cluster_msgs]
///     Support serial port and UDP communication
///
/// @author     bearboy
/// @date       ${AUTO_GENE_DATE}


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
${{message:#include <cluster_msgs/${name_lower}.h>
}}

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

${{message:// ${name_lower}
// pub
ros::Publisher      pub_${name_lower};
mavlink_${name_lower}_t     mt_pub_${name_lower};
cluster_msgs::${name_lower}     msg_pub_${name_lower};
// sub
ros::Subscriber     sub_${name_lower};
mavlink_${name_lower}_t     mt_sub_${name_lower};
cluster_msgs::${name_lower}     msg_sub_${name_lower};
            
}}


''', xml)
    
    f.close()



# generate ros_udp_pubtopic.cpp
def commnet_generate_ros_udp_pubtopic_cpp(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, "ros_udp_pubtopic.cpp"), mode='w')
    t.write(f, '''
/// @file       ros_udp_pubtopic.cpp  
/// @brief      UDP Mavlink msg => PUB ROSTopic
/// 
///     ROSTopic Pub 
///     UDP Server
///     读取参数 本机ID 仅订阅对应的端口 port
///
/// @author     bearboy
/// @date       ${AUTO_GENE_DATE}  

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
	printf("\\n TERMINATING AT USER REQUEST \\n \\n");
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
    ${{message:// Get&Pub UDP msg| ${name_lower}
    pub_${name_lower} = nh_pub.advertise<cluster_msgs::${name_lower}>("${name_lower}", 5);            
    }}
            
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
            
''', xml)
    
    
    for m in xml.message:
        t.write(f, '''        
        //-----------------------------------------------------------------------------------------
		//	${name_lower} 类      
		//-----------------------------------------------------------------------------------------
		case MAVLINK_MSG_ID_${name}:
            // 解码
			mavlink_msg_${name_lower}_decode(&mmsg, &(mt_pub_${name_lower}));
			// 赋值_common
			msg_pub_${name_lower}.sysid = mmsg.sysid;
			msg_pub_${name_lower}.compid = mmsg.compid;
			// 赋值_main
            ${{scalar_fields:msg_pub_${name_lower}.${name} = mt_pub_${name_lower}.${putname};
            }}// 发布
			pub_${name_lower}.publish(msg_pub_${name_lower});
			// rmsg pub_${name_lower}.
		break;
''', m)
    
    t.write(f, '''

        default:

		break;
	}

}
''', xml)
    
    f.close()



# generate ros_udp_subtopic.cpp
def commnet_generate_ros_udp_subtopic_cpp(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, "ros_udp_subtopic.cpp"), mode='w')
    t.write(f, '''
/// @file       ros_udp_subtopic.cpp  
/// @brief      SUB ROSTopic => UDP Mavlink msg
/// 
///     ROSTopic Sub 
///     UDP Client
///     读取参数 本机ID 往除其外的其他端口发 port
///
/// @author     bearboy
/// @date       ${AUTO_GENE_DATE}  

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
	printf("\\n TERMINATING AT USER REQUEST \\n \\n");
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
            
''', xml)
    
    for m in xml.message:
        t.write(f, '''
//-----------------------------------------------------------------------------------------
//	${name_lower} 类 
//-----------------------------------------------------------------------------------------
void cb_sub_${name_lower}(const cluster_msgs::${name_lower}::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

    ${{scalar_fields:mt_sub_${name_lower}.${name} = rmsg->${putname};
    }}
	mavlink_msg_${name_lower}_encode( my_id, rmsg->compid, &mmsg, &mt_sub_${name_lower});
    send_buffer( &mmsg );
}

''', m)

    t.write(f, '''
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

    ${{message:// get| ${name_lower}
    sub_${name_lower} = nh_sub.subscribe<cluster_msgs::${name_lower}>("${name_lower}", 5, cb_sub_${name_lower});           
    }}           

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
        std::cout << "set socket error...\\n" << std::endl;
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
''', xml)

    f.close()        



def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.command_24bit:
        xml.command_24bit_define = "1"
    else:
        xml.command_24bit_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = os.path.basename(i)[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    if not xml.command_24bit:
        for msgid in range(256):
            mlen = xml.message_min_lengths.get(msgid, 0)
            xml.message_lengths_array += '%u, ' % mlen
        xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_crcs.keys()):
            xml.message_crcs_array += '{%u, %u, %u, %u, %u, %u, %u}, ' % (msgid,
                                                                      xml.message_crcs[msgid],
                                                                      xml.message_min_lengths[msgid],
                                                                      xml.message_lengths[msgid],
                                                                      xml.message_flags[msgid],
                                                                      xml.message_target_system_ofs[msgid],
                                                                      xml.message_target_component_ofs[msgid])
    else:
        for msgid in range(256):
            crc = xml.message_crcs.get(msgid, 0)
            xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_names.keys()):
            name = xml.message_names[msgid]
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
    else:
        for msgid in range(256):
            name = xml.message_names.get(msgid, None)
            if name is not None:
                xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
            else:
                # Several C compilers don't accept {NULL} for
                # multi-dimensional arrays and structs
                # feed the compiler a "filled" empty message
                xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # form message name array
    xml.message_name_array = ''
    # sort by names
    for msgid, name in sorted(iteritems(xml.message_names), key=lambda k_v: (k_v[1], k_v[0])):
        xml.message_name_array += '{ "%s", %u }, ' % (name, msgid)
    xml.message_name_array = xml.message_name_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value
        if m.needs_pack:
            m.MAVPACKED_START = "MAVPACKED("
            m.MAVPACKED_END = ")"
        else:
            m.MAVPACKED_START = ""
            m.MAVPACKED_END = ""

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    xml.AUTO_GENE_DATE = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    # generate ros_udp_commnet.h
    commnet_generate_ros_udp_commnet_h(directory, xml)
    # generate ros_udp_pubtopic.cpp
    commnet_generate_ros_udp_pubtopic_cpp(directory, xml)
    # generate ros_udp_subtopic.cpp
    commnet_generate_ros_udp_subtopic_cpp(directory, xml)


    # generate .cpp
    #for m in xml.message:
    #    print(m)

    #generate_mavlink_h(directory, xml)
    #generate_version_h(directory, xml)
    #generate_main_h(directory, xml)
    #for m in xml.message:
    #    generate_message_h(directory, m)
    #generate_testsuite_h(directory, xml)



def generate(basename, xml_list):
    '''generate complete custom C++ header file'''

    for idx in range(len(xml_list)):
        xml = xml_list[idx]
        xml.xml_idx = idx
        generate_one(basename, xml)
    #copy_fixed_headers(basename, xml_list[0])