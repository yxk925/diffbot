#!/usr/bin/env python

from lx98n_driver_wiringpi import Lx98N_MotorDriverWiringpi

import rospy
from std_msgs.msg import Int32


class MotorDriver:

    def __init__(self, address=0x0f):
        self.motor_driver = Lx98N_MotorDriverWiringpi()
        #"0b1001" defines the output polarity of both motors
        # The first two least significant bits "01" are for the right motor 
        # and the following bits "10" specify the polarity of the left motor,
        # "10" means the M+ is "positive
        self.motor = {
            'left': 0,
            'right': 0,
        }

    def left_motor_callback(self, msg):
        rospy.loginfo_once("left_motor_callback")
        self.motor['left'] = msg.data

        self.update_motors()


    def right_motor_callback(self, msg):
        rospy.loginfo_once("right_motor_callback")
        self.motor['right'] = msg.data

        self.update_motors()


    def update_motors(self):
        left = self.motor['left']
        right = self.motor['right']
        rospy.logdebug(format("update_motors, left: {left}, right: {right}"))
        self.motor_driver.MotorSpeedSetAB(left, right)

    def stop_motors(self):
        self.motor_driver.MotorSpeedSetAB(0, 0)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('motor_driver')

    motor_driver = MotorDriver(address=0x0f)
    motor_driver.stop_motors();

    # Topics for the motors
    sub_left = rospy.Subscriber('motor_left', Int32, motor_driver.left_motor_callback, queue_size=10)
    sub_right = rospy.Subscriber('motor_right', Int32, motor_driver.right_motor_callback, queue_size=10)

    rospy.loginfo("[Lx98N_driver]Ready to receive motor commands")

    # Start everything
    # This will block until the node is killed
    # Callbacks run in their own thread and will still work
    rospy.spin()

    rospy.loginfo("Stop motors")
    motor_driver.stop_motors()
