#!/usr/bin/env python3
"""rcfw_controller_node.py - simple heartbeat node for rcfw_controller package"""

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('rcfw_controller')
    pub = rospy.Publisher('rcfw_heartbeat', String, queue_size=1)
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo('rcfw_controller node started')
    while not rospy.is_shutdown():
        pub.publish('alive')
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
