#!/usr/bin/env python
"""
Test script that should be able to read the angle from the ngimu/euler node
and use that angle to calculate how much to turn the sub. It should then
guide the sub towards the target more easily.
"""
from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time
# possible bug source
# I'm trying to read Vector3 and Vector3Stamped messages over ROS
# might have to change these a bit to make them work
# find where the geometry_msgs folder is and look for the message declaration
from geometry_msgs.msg import Vector3, Vector3Stamped

import ros_utils

desired_yaw = 0
current_yaw = 0
frame_width = 640
frame_height = 480
default_power = 0.5
h_fov = 80 #degrees
v_fov = 64 #degrees


def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   try:
        global desired_yaw, current_yaw, current_distance, frame_width, frame_height, default_power
        json_data = json.loads(data.data)
        xmin = json_data['xmin']
        print("got xmin from json: " + str(xmin))
        xmax = json_data['xmax']
        print("got xmax from json: " + str(xmax))
        ymin = json_data['ymin']
        print("got ymin from json: " + str(ymin))
        ymax = json_data['ymax']
        print("got ymax from json: " + str(ymax))

        object_center = (xmax+xmin)/2
        # calculate desired yaw
        degree_displacement = calc_angle(object_center)
        desired_yaw = calc_desired_yaw(object_center, current_yaw, frame_width, frame_height, h_fov)
        motors_client(desired_yaw, default_power)
   except KeyboardInterrupt:
        set_disabled()

def cv_controls_test():
    try:
        rospy.init_node('cv_controls_test',anonymous = True)
        rospy.Subscriber("cv_detection", String, callback)
        rospy.Subscriber('ngimu/euler', Vector3Stamped, get_imuangle)
        rospy.spin()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

# def zero_yaw():
#     print("Waiting for Zero Service")
#     rospy.wait_for_service('motion_controller/Zero')
#     print("After waiting for Zero service")
#
#     try:
#         zero = rospy.ServiceProxy('motion_controller/Zero', Zero)
#         call_zero = zero()
#         print(call_zero)
#         return
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

# def set_disabled():
#     print("Waiting for SetEnabled")
#     rospy.wait_for_service('motion_controller/SetEnabled')
#     print("After SetEnabled")
#
#     try:
#         set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
#         enable = set_enabled(False)
#         return
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e


if __name__ == '__main__':
    try:
        # zero_yaw()
        cv_Controls_Test()
    except (KeyboardInterrupt, SystemExit):
        print("Exiting")
        set_disabled()
