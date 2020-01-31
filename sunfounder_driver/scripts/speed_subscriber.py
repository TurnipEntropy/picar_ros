#!/usr/bin/env python
import rospy
from std_msgs.msg import Byte
from SunFounder_PiCar.picar import back_wheels

def callback(data):
    rospy.loginfo('received speed: {}'.format(data.data))
    bwheels = back_wheels.Back_Wheels()

def listener():
    rospy.init_node('speed_listener', anonymous=True)
    rospy.Subscriber('pwm', Byte, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()