#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import curses
import tty
import sys
import select
import termios


class KeyboardControl:
    def __init__(self):
        print(rospy.get_param_names())
        self.pub_name = rospy.get_param('keyboard_topic')
        self.pub = rospy.Publisher(self.pub_name, String, queue_size=10)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def start_tty(self):
        try:
            while True:
                key = self.get_key()
                if key == '\x03':
                    break
                self.pub.publish(key)
        except Exception, e:
            print(e)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))



if __name__ == '__main__':
    rospy.init_node(rospy.get_param('key_node'))
    controller = KeyboardControl()
    controller.start_tty()
