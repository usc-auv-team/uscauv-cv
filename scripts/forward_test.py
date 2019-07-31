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
    #time.sleep(30)
    try:
        rospy.init_node('forwards_test',anonymous = True)
        print("Before calling forwards")
        ru.forwards(60 , 1)
        # ru.motors_client(0, 1)
        print("End of forwards script")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")
    return

if __name__ == '__main__':
    main()
