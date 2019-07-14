#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Vector3Stamped


class MotorController():
    """
    Intantiates a controller for the moters to avoid interfacing with them directly.
    This way the PID loop won't keep jamming.
    """

    def __init__(self):
        # super(MotorController, self).__init__()
        self.yaw = 0
        self.yaw_storage = []
        self.polling_rate = 100

    def is_yaw_moving(self):
        rospy.init_node('motor_listener', anonymous=True)
        rospy.Subscriber("ngimu/euler", Vector3Stamped, self.is_yaw_moving_callback)

        rospy.spin()

    def is_yaw_moving_callback(self, data):
        # Get current yaw
        curr_yaw = data.vector.z # This is the new yaw from data

        # Check it with the old yaw
        # Get speed from the change in the angle
        yaw_speed = (curr_yaw - self.yaw) / 100 # Rotation speed in degrees/second
        self.yaw = curr_yaw
        # Store the yaw in a list and only operate every 20 frames
        self.yaw_storage.append(curr_yaw)

        print(f"Yaw Speed: {round(yaw_speed, 3)}")
        # if yaw_speed > 1:
            # print("turning yaw")
        if(len(self.yaw_storage) >= 20):
            # Calculate variance of list
            mean = sum(self.yaw_storage) / len(self.yaw_storage)
            variance = sum((xi - mean) ** 2 for xi in self.yaw_storage) 
            # print(f"Variance: {variance}")
            self.yaw_storage.clear()
            # if variance > 1:
                # print("turning var")
        # If the variance is close to 0, we are stable
        # If the variance is high, our yaw is changing

        # Based on calculations, print whether moving or still
        # --


def main():
    print("Main started!")
    motor_controller = MotorController()
    motor_controller.is_yaw_moving()

if __name__ == '__main__':
    main()
