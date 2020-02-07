#!/usr/bin/env python
import rospy
from std_msgs.msg import Byte
from SunFounder_PiCar.picar import front_wheels
from SunFounder_PiCar import picar
from sunfounder_driver.msg import Steering

class FrontWheels:

    def __init__(self):
        self.fwheels = front_wheels.Front_Wheels(debug=False)
        picar.setup()

    def callback(self, data):
        rospy.loginfo('received steering: {}'.format(data.data))
        # self.bwheels.speed = data.data
        # self.bwheels.forward()

    def listener(self):
        rospy.init_node('steering_listener', anonymous=True)
        rospy.Subscriber('steering', Steering, self.callback)
        rospy.spin()


if __name__ == '__main__':
    fw = FrontWheels()
    fw.listener()