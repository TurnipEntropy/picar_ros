#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped as AckDS
from std_msgs.msg import Int32, String, Header


class Mux:
    def __init__(self):
        self.node_name = rospy.get_param("mux_node")
        self.mux_topic = rospy.get_param("mux_topic")
        self.drive_topic = rospy.get_param("drive_topic")
        self.key_topic = rospy.get_param("keyboard_topic")
        self.max_pwm = rospy.get_param('max_pwm')
        self.min_pwm = rospy.get_param('min_pwm')

        self.drive_pub = rospy.Publisher(self.drive_topic, AckDS, queue_size=10)

        # Changed the Int32MultiArray into an Int32 since it can just be a bit array
        self.mux_sub = rospy.Subscriber(self.mux_topic, Int32, self.mux_callback)
        self.key_sub = rospy.Subscriber(self.key_topic, String, self.key_callback)

        self.mux_bit_arr = int(0)
        self.prev_mux = int(0)
        self.key_mux_idx = rospy.get_param('key_mux_idx')

        self.speed_adj = 15
        self.steer_adj = 7.5  # degrees, that's what the picar works with
        self.ads = AckDS()

    def mux_callback(self, mux_int):
        changed = False
        anything_on = False
        self.mux_bit_arr = mux_int.data
        if mux_int.data != self.prev_mux:
            changed = True
            anything_on = True

        if changed:
            rospy.loginfo("Mux: ")
            rospy.loginfo(mux_int)

        if not anything_on:
            self.publish_to_drive(0.0, 0.0)

        self.prev_mux = mux_int.data

    def publish_to_drive(self, vel, steer):
        header = Header()
        header.stamp = rospy.Time.now()
        self.ads.header = header
        self.ads.drive.speed = vel
        self.ads.drive.steering_angle = steer
        self.drive_pub.publish(self.ads)

    def key_callback(self, msg):
        if self.mux_bit_arr & (1 << self.key_mux_idx):
            vel = self.ads.drive.speed
            steer = self.ads.drive.steering_angle
            publish = True
            if msg.data == 'w':
                vel += self.speed_adj
                if 0 <= vel < self.min_pwm:
                    vel = self.min_pwm
                elif -1 * self.min_pwm < vel < 0:
                    vel = 0.0
                elif vel > self.max_pwm:
                    vel = self.max_pwm
            elif msg.data == 's':
                vel -= self.speed_adj
                if 0 < vel < self.min_pwm:
                    vel = 0.0
                elif -1 * self.min_pwm < vel <= 0:
                    vel = -1 * self.min_pwm
                elif vel < -1 * self.min_pwm:
                    vel = -1 * self.min_pwm
            elif msg.data == 'a':
                steer -= self.steer_adj
            elif msg.data == 'd':
                steer += self.steer_adj
            elif msg.data == ' ':
                vel = 0.0
                steer = 0.0
            else:
                if abs(vel) <= self.min_pwm:
                    vel = 0.0
                else:
                    vel = int(vel / 1.15)

            if publish:
                self.publish_to_drive(vel, steer)


if __name__ == '__main__':
    rospy.init_node(rospy.get_param('mux_node'))
    mux = Mux()
    rospy.spin()
