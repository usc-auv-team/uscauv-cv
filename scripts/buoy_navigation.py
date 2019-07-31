#!/usr/bin/env python
"""
Should be able to control the whole sub
"""
from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time
import ros_utils as ru

def nav_to_buoy():
    try:
        rospy.init_node('buoy_nav',anonymous = True)
        rospy.Subscriber("cv_detection", String, callback)
        #rospy.Subscriber('ngimu/euler', Vector3Stamped, get_imuangle)
        rospy.spin()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

if __name__ == '__main__':
    nav_to_buoy()
