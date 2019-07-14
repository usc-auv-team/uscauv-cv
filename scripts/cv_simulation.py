#!/usr/bin/env python
"""
USCAUV CV Simulation
Provides simulated CV data over the same pathway that cv_inference does
"""
import sys, signal
import time

def signal_handler(signal, frame):
    print("\nExiting program.")
    sys.exit(0)

import rospy
from std_msgs.msg import String


def simulation():
    
    #setting up signal handler to exit with Ctrl-C
    signal.signal(signal.SIGINT, signal_handler)

    # Setup the publisher
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
    
    while True:
        sent_str = "{" + "\"xmin\":" + str(xmin) + ", \"xmax\":" + str(xmax) + ", \"ymin\":" + str(ymin) + ", \"ymax\":" + str(ymax) + "}"
        pub.publish(sent_str)
        xmin = xmin + direction
        xmax = xmax + direction
        if(xmin == frame_width or xmax == frame_width):
            direction = direction * -1
        time.sleep(1)
      
if __name__=='__main__':
        simulation()
