#!/usr/bin/env python
import rospy
from motion_controller.srv import * 
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String

def motors_client(angle, power):
    print("Before SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    print("After SetEnabled, before Set ForwardsPower")
    rospy.wait_for_service('motion_controller/SetForwardsPower')
    print("After SetFowardsPower, before SetYawAngle")
    rospy.wait_for_service('motion_controller/SetYawAngle')
    print("After SetYawAngle")

    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        set_forwards_power = rospy.ServiceProxy('motion_controller/SetForwardsPower', SetForwardsPower)
        set_yaw_angle = rospy.ServiceProxy('motion_controller/SetYawAngle', SetYawAngle)
        m_power = set_forwards_power(power)
        m_angle = set_yaw_angle(angle)
        # m_enable = set_enabled(True)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_imuyaw():
    #Return yaw/z value from the imu
    data = rospy.wait_for_message('ngimu/euler', Vector3Stamped)
    current_yaw = data.vector.z
    return current_yaw


def calc_desired_yaw(object_center, current_yaw, frame_width, frame_height, h_fov):
    """ Based of previous "get_imuangle function. Now, the funtion returns the desired yaw as opposed to a differential.
    NOTE: This math might give wrong values but should give right
    or left correctly"""

    # getcontext().prec = 6

    frame_w_center = frame_width/2
    frame_h_center = frame_height/2
    pixel_displacement = object_center - frame_w_center
    pixel_fraction = pixel_displacement/(frame_width/2.0)
    degree_displacement = (h_fov/2) * pixel_fraction
    desired_yaw = current_yaw + degree_displacement

    print("----")
    print("Calculation Values:")
    print("Object Center: " + str(object_center))
    print("Current Yaw: " + str(current_yaw))
    print("Frame Width: " + str(frame_width))
    print("Frame Height: " + str(frame_height))
    print("H FOV: " + str(h_fov))
    print("Object Center: " + str(object_center))
    print("Pixel Displacement: " + str(pixel_displacement))
    print("Pixel Fraction: " + str(pixel_fraction))
    print("Degree Displacement: " + str(degree_displacement))

    print("----")


    return desired_yaw
