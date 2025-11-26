#!/usr/bin/env python3
"""rcfw_controller_measured.py

Subscribe to wheel_cmd_velocities (diffbot_msgs/WheelsCmdStamped) and publish
sensor_msgs/JointState on topic measured_joint_states.

This mirrors the C++ RcfwController behavior but implemented in Python.
"""

import rospy
import threading
import serial
from sensor_msgs.msg import JointState
from diffbot_msgs.msg import WheelsCmdStamped


def to_rpm(wheel_speed):
    return wheel_speed * 60.0 / (2.0 * 3.141592653589793)

def to_rad_per_sec(wheel_rpm):
    return wheel_rpm * (2.0 * 3.141592653589793) / 60.0

class RcfwController(object):
    def __init__(self):

        # default joint names (can be overridden via ROS param)
        self.joint_names = rospy.get_param('~joint_names', ['wheel_left_joint', 'wheel_right_joint'])

        # publisher for measured joint states
        self.pub_measured = rospy.Publisher('measured_joint_states', JointState, queue_size=10)

        # subscriber for wheel command velocities
        self.sub_cmd = rospy.Subscriber('wheel_cmd_velocities', WheelsCmdStamped, self.wheel_cmd_callback, queue_size=10)

        # initialize JointState message
        self.measured_msg = JointState()
        self.measured_msg.name = self.joint_names
        n = len(self.joint_names)
        self.measured_msg.position = [0.0] * n
        self.measured_msg.velocity = [0.0] * n
        self.measured_msg.effort = [0.0] * n

        rospy.loginfo('RcfwControllerPy initialized: publishing "measured_joint_states" and subscribing to "wheel_cmd_velocities"')
       
        self.serial_port          = serial.Serial()
        self.serial_port.port     = '/dev/ttyS0'
        self.serial_port.baudrate = 19200
        self.serial_port.timeout  = 60
        self.serial_port.open()
                 
        self.read_thread = threading.Thread(target = self.read_thread_function, args = ())
        self.read_thread.start()

    def wheel_cmd_callback(self, msg: WheelsCmdStamped):
        # guard: ensure there are enough joint velocities in the message
        try:
            incoming = msg.wheels_cmd.angular_velocities.joint
        except Exception:
            rospy.logwarn_throttle(5.0, 'Received wheel_cmd_velocities without expected structure')
            return

        if len(incoming) < len(self.joint_names):
            rospy.logwarn_throttle(5.0, 'wheel_cmd_velocities has %d entries but %d expected', len(incoming), len(self.joint_names))
            return

        self.apply_velocity(incoming)
        
        return
        
    
    def apply_velocity(self, velocities):

        rospy.loginfo('Applying velocity command %.2f, %.2f ', velocities[0], velocities[1])

#        if self.start_time == 0.0:

#            self.start_time = time.time()

        wheel_front_left  = to_rpm(velocities[0])
        wheel_front_right = to_rpm(velocities[1])
        wheel_rear_left   = 0
        wheel_rear_right  = 0

        command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)
        command_bytes  = bytes(command_string, encoding = 'ascii')

        rospy.loginfo('Sending : ' + str(command_bytes))
        self.serial_port.write   (command_bytes)

#        elif time.time() - self.start_time > 3.0:

#            command_string = 'C0 0 0 0\r'
#            command_bytes  = bytes(command_string, encoding = 'ascii')
#            self.serial_port.write(command_bytes)
#            self.get_logger().info('Sending : ' + str(command_bytes))

        return

    def publish_wheels_state(self, left_speed, right_speed, left_position, right_position):

        rospy.loginfo('Publishing wheels state pos: %.2f, %.2f speed: %.2f, %.2f', left_position, right_position, left_speed, right_speed)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [left_position, right_position]
        msg.velocity = [to_rad_per_sec(left_speed), to_rad_per_sec(right_speed)]
        self.pub_measured.publish(msg)
        return

    def read_thread_function(self):

        rospy.loginfo('Starting reading thread')

        
        char = None
        msg  = ''

        while not rospy.is_shutdown():

            char = self.serial_port.read(1)
            # rospy.loginfo('Read char: ' + str(char))
            
            if char == b'\r':
                split_msg = msg[1:].split()
                if msg[0] == 'S' and len(split_msg) == 4:
                    rospy.loginfo('Receied : ' + msg)
                    try:
                        self.publish_wheels_state(float(split_msg[1]), float(split_msg[0]), float(split_msg[3]), float(split_msg[2]))
                    except Exception as e:
                        rospy.logerr('Error parsing message values. ' + str(e))
                    pass
                else:
                    rospy.loginfo('Discarding malformed message:' + msg)
                msg = ''
            elif char == b'\n':
                pass
            else:

                msg += char.decode('utf-8', 'ignore')

        rospy.loginfo('Ending reading thread')
        
        return
