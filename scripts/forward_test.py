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
    ru.forwards(5, 0.5)

if __name__ == '__main__':
    main()
