#!/usr/bin/env python
#
# Turn the Mavros topic into a topic that needs to be sent through UDP.
# GPS_data

from __future__ import division

import rospy
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from cluster_msgs.msg import global_gps_double

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Header

class Mavros2UDP(object):

    def __init__(self):
        self.ID_ALL=99
        self.my_id=100
        self.msg_mavros_global_position = NavSatFix()
        self.msg_cluster_global_gps_double = global_gps_double()
        self.flag_global_position = 0


    def callback_global_position(self, data):
        # rospy.loginfo(rospy.get_caller_id() + '| I heard Data')
        self.msg_mavros_global_position = data
        self.flag_global_position = 1


    def run(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('mavros2udpdata_gps', anonymous=True)
    
        # Gets parameter | /mavros_gps_data/my_id
        self.my_id = rospy.get_param("/mavros_gps_data/my_id", default=100)
        rospy.loginfo('Get param [/mavros_gps_data/my_id] = %d', self.my_id)
        
    
        rospy.Subscriber('mavros/global_position/global', NavSatFix, self.callback_global_position)
        gps_pub = rospy.Publisher('commnet/send/global_gps_double', global_gps_double, queue_size=1)
    
        rate = rospy.Rate(10) # 10hz    
        while not rospy.is_shutdown():
    
            if ( self.flag_global_position == 1 ):
                # rospy.loginfo(rospy.get_caller_id() + '| Try to SEND Data')
                self.msg_cluster_global_gps_double.sysid = self.my_id
                self.msg_cluster_global_gps_double.compid = self.ID_ALL
                self.msg_cluster_global_gps_double.latitude = self.msg_mavros_global_position.latitude
                self.msg_cluster_global_gps_double.longitude = self.msg_mavros_global_position.longitude
                self.msg_cluster_global_gps_double.altitude = self.msg_mavros_global_position.altitude
                
                gps_pub.publish(self.msg_cluster_global_gps_double)
            
            rate.sleep()
        
if __name__ == '__main__':

    t = Mavros2UDP()
    
    try:
        t.run()
    except rospy.ROSInterruptException:
        pass
        
