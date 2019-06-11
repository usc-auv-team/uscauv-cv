#!/usr/bin/env python
from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json

def cv_Motors_Client(angle, power):
    rospy.wait_for_service("SetEnabled")
    rospy.wait_for_service("SetForwardsPower")
    rospy.wait_for_service("SetYawAngle")

    try:
        set_enabled = rospy.ServiceProxy("SetEnabled", setEnabled)
        set_forwards_power = rospy.ServiceProxy("SetForwardsPower", setForwardsPower)
        set_yaw_angle = rospy.ServiceProxy("SetYawAngle", setYawAngle)
        power = set_forwards_power(power)
        angle = set_yaw_angle(angle)
        enable = set_enabled(true)

        #if(power == 0):
        #    enable = set_enabled(false)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    cv_Controls_Test()

