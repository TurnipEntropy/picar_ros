#!/usr/bin/env python
import rospy
from std_msgs.msg import Byte
from SunFounder_PiCar.picar import back_wheels

class BackWheels:

    def __init__(self):
        self.bwheels = back_wheels.Back_Wheels(debug=False)
        self.bwheels.forward()

    def callback(self, data):
        rospy.loginfo('received speed: {}'.format(data.data))
        self.bwheels.speed = data.data

    def listener(self):
        rospy.init_node('speed_listener', anonymous=True)
        rospy.Subscriber('pwm', Byte, self.callback)
        rospy.spin()


if __name__ == '__main__':
    bw = BackWheels()
    bw.listener()