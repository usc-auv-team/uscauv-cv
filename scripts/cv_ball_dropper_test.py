#!/usr/bin/env python

from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time
import subprocess.

def callback(data):
    subprocess.call(["g++", "I2C_Ball_Drop.cpp"])
    subprocess.call("./I2C_Ball_Drop")

def cv_Ball_Dropper_Test():
    try:
        rospy.init_node("cv_Ball_Dropper_Test",anonymous = True)
        rospy.Subscriber("cv_detection", String, callback)
        rospy.spin()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

if __name__ == '__main__':
    try:
        cv_Ball_Dropper_Test()
    except (KeyboardInterrupt, SystemExit):
        print("Exiting")
