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
        down = 330
        forw = 335
        delta = -2
        rospy.init_node('forwards_test',anonymous = True)
        print("Before calling forwards")
        #ru.forwards(seconds=5, power=1, wait=0, roll=0, pitch=320)
        #ru.forwards(seconds=5, power=1, wait=30, roll=0, pitch=320)
        #yaw = ru.get_imuyaw()
        #print("Yaw:")
        #print(yaw)
        #ru.set_yaw(yaw)
        ru.forwards_start(seconds=10, power=1, wait=30, roll=0, pitch=down)
        yaw = ru.get_imuyaw()
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=down)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=down)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards_int(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        y = ru.get_imuyaw()
        print(y)
        yaw = yaw - delta
        ru.set_yaw(yaw)
        ru.forwards(seconds=10, power=1, wait=0, roll=0, pitch=forw)
        #ru.forwards(seconds=5, power=1, wait=0, roll=0, pitch=320)
        #ru.forwards(seconds=10, power=1, wait=0, roll=0, pitch=325)		
        # ru.motors_client(0, 1)
        print("End of forwards script")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")
    return

if __name__ == '__main__':
    main()
