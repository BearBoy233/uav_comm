#!/usr/bin/env python
#
# Turn the Mavros topic into a topic that needs to be sent through UDP.
# GPS_data

from __future__ import division

# PKG = 'px4'

import rospy
import math
import numpy as np
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Header

global sself_data
sself_data = "NaN"

def callback(data):
    global sself_data
    
    # rospy.loginfo(rospy.get_caller_id() + '| I heard %s', data.data)
    sself_data = data.data


def run():
    global sself_data

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sample_subpub', anonymous=True)

    rospy.Subscriber('chatter', String, callback)
    pub = rospy.Publisher('chatter222', String, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo(hello_str)
        pub.publish(sself_data)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
        
