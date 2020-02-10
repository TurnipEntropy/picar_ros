#!/usr/bin/env python
import rospy
from std_msgs.msg import Byte
from sunfounder_driver.msg import Steering

def speed_publisher():
    pub = rospy.Publisher('steering', Steering, queue_size=5)
    rospy.init_node('steer_pub', anonymous=True)
    rate = rospy.Rate(10)
    i = 0
    increase = True
    while not rospy.is_shutdown():
        rospy.loginfo('speed sent to pwm: {}'.format(i))
        direction = raw_input('enter desired direction:\n')
        print(direction)
        steering = Steering()
        if direction == 'w':
            steering.direction = 'straight'
            steering.angle = 0
        elif direction == 'a':
            steering.direction = 'left'
            steering.angle = 10.
        elif direction == 'd':
            steering.direction = 'right'
            steering.angle = 10.
        pub.publish(steering)
        rate.sleep()


if __name__ == '__main__':
    try:
        speed_publisher()
    except rospy.ROSInternalException:
        pass
