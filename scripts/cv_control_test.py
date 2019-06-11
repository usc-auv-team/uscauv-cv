#!/usr/bin/env python

from motion_controller.srv import * 
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
import rospy
import re
import json
import time

rate = None
can_update = True
current_angle = 200
current_distance = 0
frame_w_center = 640/2
frame_h_center = 480/2
default_power = 0.25
last_time = None
current_time = None

def cv_Motors_Client(angle, power):
    
    print("Before SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    print("After SetEnabled, before Set ForwardsPower")
    rospy.wait_for_service('motion_controller/SetForwardsPower')
    print("After SetFowardsPower, before SetYawAngle") 
    rospy.wait_for_service('motion_controller/SetYawAngle')
    print("After SetYawAngle")
    print("Getting Angle from ngimu");
    
    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        set_forwards_power = rospy.ServiceProxy('motion_controller/SetForwardsPower', SetForwardsPower)
        set_yaw_angle = rospy.ServiceProxy('motion_controller/SetYawAngle', SetYawAngle)
        #m_power = set_forwards_power(power)
        m_angle = set_yaw_angle(angle)
        #m_enable = set_enabled(True)
        rate.sleep()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def update_angle(vector):
    global can_update
    if(can_update):
        roll, pitch, yaw = vector
        current_angle = yaw
        if(current_angle < 0):
            current_angle += 360

def callback(data):
    global current_time, last_time
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    can_update = False
    current_time = rospy.get_rostime()
    global current_angle, current_distance, frame_w_center, frame_h_center, default_power
    json_data = json.loads(data.data) 
    xmax = json_data['xmax']
    print("got xmax from json: " + str(xmax))
    xmin = json_data['xmin']
    print("got xmin from json: " + str(xmin))
    object_center = (xmax+xmin)/2
    power = 0
    if(current_time - last_time  > rospy.Duration(5)):
        current_time = last_time
        if(abs(object_center - frame_w_center) < 10):
            power = default_power                
        elif(object_center > frame_w_center):
            current_angle+=1
	    if(current_angle > 360):
		current_angle = current_angle % 360
            print("Setting to " + str(current_angle))
            print("moving right")
        elif(object_center < frame_w_center):
    	    print("Setting to " + str(current_angle))
	    print("moving left")
	    current_angle-=1
	    if(current_angle > 0):
                current_angle += 360
            cv_Motors_Client(current_angle, default_power)
    can_update = True

def cv_Controls_Test():
    global current_time, last_time, rate
    rospy.init_node('cv_Controls_Test',anonymous = True)
    current_time = rospy.get_rostime()
    last_time = rospy.get_rostime()
    rate = rospy.Rate(5)
    rospy.Subscriber("cv_detection", String, callback)
    rospy.Subscriber("euler", Vector3Stamped, update_angle)
    rospy.spin()

def zero_yaw():
    print("Waiting for Zero Service")
    rospy.wait_for_service('motion_controller/Zero')
    print("After waiting for Zero service")

    try:
        zero = rospy.ServiceProxy('motion_controller/Zero', Zero)
        call_zero = zero()
        print(call_zero)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_disabled():
    print("Waiting for SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    print("After SetEnabled")

    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        enable = set_enabled(False)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    try:
	#zero_yaw()
        cv_Controls_Test()
    except (KeyboardInterrupt, SystemExit):
        set_disabled()
