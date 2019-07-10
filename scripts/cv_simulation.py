#!/usr/bin/env python
"""
USCAUV CV Simulation
Provides simulated CV data over the same pathway that cv_inference does
"""
import numpy as np
import os
import sys
import tensorflow as tf
import time

import cv2

from collections import defaultdict
from io import StringIO
from PIL import Image

import rospy
from std_msgs.msg import String


def simulation():
    # Setup the phublisher
    refresh_rate = 10
    pub = rospy.Publisher('cv_detection', String, queue_size=10)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(refresh_rate) #10hz rate, change if needed
    # Not sure if I need seq_if but if I do, I want to be 42
    seq_id = 42

    frame_width = 640
    direction = 1

    xmin = 0
    xmax = 0
    ymin = 20
    ymax = 50
    try:
        while True:

            sent_str = "{" + "\"xmin\":" + str(xmin) + ", \"xmax\":" + str(xmax) + ", \"ymin\":" + str(ymin) + ", \"ymax\":" + str(ymax) + "}"
            pub.publish(sent_str)
            xmin = xmin + direction
            xmax = xmax + direction
            if(xmin == frame_width || xmax == frame_width):
                direction = direction * -1
            time.sleep(2)
    except KeyboardInterrupt:
        pass

if __name__=='__main__':
    try:
        simulation()
    except KeyboardInterrupt:
        pass
