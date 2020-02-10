import rospy
from SunFounder_PiCar.picar import front_wheels, back_wheels
from SunFounder_PiCar import picar
from geometry_msgs.msg import Twist


class MovementSubscriber:

    def __init__(self):
        self.fwheels = front_wheels.Front_Wheels(debug=False)
        self.bwheels = back_wheels.Back_Wheels(debug=False)
        picar.setup()

    def callback(self, data):
        """
        :param data: type: Twist
        :return:
        """
        rospy.loginfo('received data: {}'.format(data))
        speed = data.linear.x
        angle = data.angular.x
        cur_angle = self.fwheels.wheel.angle
        target_angle = cur_angle + angle
        cur_speed = self.bwheels.speed
        if speed == 0.5:
            target_speed = cur_speed * speed
        elif speed == 0:
            target_speed = 0.
            target_angle = 90.
        else:
            target_speed = cur_speed + speed
            if 25 >= target_speed >= 0 and speed > 0:
                target_speed = 25
            elif 0 >= target_speed >= -25 and speed > 0:
                target_speed = 0
            elif 0 >= target_speed >= -25 and speed < 0:
                target_speed = -25
            elif 0 >= target_speed >= -25 and speed > 0:
                target_speed = 0

        self.fwheels.turn(target_angle)
        self.bwheels.speed = abs(target_speed)
        if target_speed < 0:
            self.bwheels.forward()  # current bug in config, forward = backward, backward = forward
        else:
            self.bwheels.backward()

    def listener(self):
        rospy.init_node('movement_listener', anonymous=True)
        rospy.Subscriber('picar_teleop', Twist, self.callback)
        rospy.spin()


if __name__ == '__main__':
    controller = MovementSubscriber()
    controller.listener()


