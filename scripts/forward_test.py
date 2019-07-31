#!/usr/bin/env python
"""
Test script that tries to run the motors for a specified period of time
"""
from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time
import ros_utils as ru

def main():
    try:
        rospy.init_node('forwards_test',anonymous = True)
        ru.forwards(5, 0.5)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

if __name__ == '__main__':
    main()
