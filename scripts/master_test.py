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

def main():
    # First run the sub forwards until we pass the gate
    ru.forwards(5, 0.5)

    # Then navigate towards the buoy
    nav = True
    missed = 0
    while(nav)
        data = rospy.wait_for_message('cv_detection', String, timeout = 3.0)
        if data is None:
            missed += 1
        if missed == 5:
            nav = False

    # Backup and try to go past to the other obstacle
    ru.forwards(5,-0.5)

    data = rospy.wait_for_message('ngimu/euler', Vector3Stamped)
    current_yaw = data.vector.z
    ru.motors_client(current_yaw + 10, 0.5)

    return


if __name__ == '__main__':
    main()
