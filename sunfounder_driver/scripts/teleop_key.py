#!/usr/bin/env python

import select
import sys
import termios
import tty
import time

import rospy
from geometry_msgs.msg import Twist


class KeyboardControl:
    def __init__(self, pub_settings, settings):
        self.bindings = {
            'w': (3, 0.),
            'a': (0., -7.5),
            'd': (0., 7.5),
            's': (-3, 0.),
            ' ': (0., 90.)
        }
        self.settings = settings
        rospy.init_node(pub_settings['node_name'])
        self.pub = rospy.Publisher(pub_settings['topic_name'],
                                   pub_settings['msg_type'], queue_size=pub_settings['queue_size'])
        self.count = 0
        self.last_command = (1.0, 0.)
        self.last_command_time = 0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 3)
        if rlist:
            key = sys.stdin.read(0.1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def start_tty(self):
        try:
            while True:
                key = self.get_key()
                if key in self.bindings:
                    target_adj = self.bindings[key]
                    while time.time() - self.last_command_time < 3:
                        continue
                    if key == 'w' or key == 's':
                        self.last_command = target_adj
                    self.count = 0
                else:
                    self.count += 1
                    if self.count > 2:
                        target_adj = self.bindings[' ']
                    else:
                        target_adj = self.last_command[0] / abs(self.last_command[0]) * 0.5, 0.
                        # target_adj = (0., 0.)  # TODO: change to slow car down and turn straighter
                    if key == '\x03':
                        break
                self.last_command_time = time.time()
                msg = Twist()
                msg.linear.x = target_adj[0]
                msg.linear.y = msg.linear.z = 0.
                msg.angular.x = target_adj[1]
                msg.angular.y = msg.angular.z = 0.
                print('sending: {}'.format(msg))
                self.pub.publish(msg)
        except Exception as e:
            print(e)
        finally:
            target_adj = self.bindings[' ']
            msg = Twist()
            msg.linear.x = target_adj[0]
            msg.linear.y = msg.linear.z = 0.
            msg.angular.x = target_adj[1]
            msg.angular.y = msg.angular.z = 0.
            self.pub.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    pub_settings = {
        'node_name': 'picar_teleop',
        'topic_name': 'picar_cmd_vel',
        'msg_type': Twist,
        'queue_size': 5
    }
    keyteleop = KeyboardControl(pub_settings, settings)
    keyteleop.start_tty()
