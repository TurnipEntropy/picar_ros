#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from SunFounder_PiCar.picar import front_wheels, back_wheels
from SunFounder_PiCar import picar


class DriveController:
    def __init__(self):
        self.fwheels = front_wheels.Front_Wheels(debug=False)
        self.bwheels = back_wheels.Back_Wheels(debug=False)
        picar.setup()

    def callback(self, data):
        """
        :param data: type: AckermannDriveStamped
        :return:
        """
        rospy.loginfo('received data: {}'.format(data))
        speed = data.drive.speed
        angle = data.drive.steering_angle

        self.fwheels.turn(angle)
        self.bwheels.speed = abs(speed)
        if speed < -0.1:
            self.bwheels.backward()
        else:
            self.bwheels.forward()

    def listener(self):
        rospy.init_node('movement_listener', anonymous=True)
        rospy.Subscriber('pidrive', AckermannDriveStamped, self.callback)
        rospy.spin()


if __name__ == '__main__':
    controller = DriveController()
    controller.listener()
