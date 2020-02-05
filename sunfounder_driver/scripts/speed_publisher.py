#!/usr/bin/env python
import rospy
from std_msgs.msg import Byte


def speed_publisher():
    pub = rospy.Publisher('pwm', Byte, queue_size=5)
    rospy.init_node('speed_pub', anonymous=True)
    rate = rospy.Rate(0.25)
    i = 0
    increase = True
    while not rospy.is_shutdown():
        rospy.loginfo('speed sent to pwm: {}'.format(i))
        pub.publish(i)
        if i == 100:
            increase = False
        elif i == 0:
            increase = True

        if increase:
            i += 10
        else:
            i -= 10
        rate.sleep()


if __name__ == '__main__':
    try:
        speed_publisher()
    except rospy.ROSInternalException:
        pass
