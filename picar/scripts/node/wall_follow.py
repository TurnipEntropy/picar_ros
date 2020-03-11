#!/usr/bin/env python

import math
import numpy as np
import time
import signal

# ROS Imports
import rospy
from rospy import Subscriber, Publisher, Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow:
    def __init__(self, params=(0.9, 0.0008, 0.0008), angle_speeds=None):
        if angle_speeds is None:
            angle_speeds = {20: 35,
                            10: 55,
                            0: 75
                            }
        lidar_topic = '/scan'
        drive_topic = '/drive'
        self.lidar_sub = Subscriber(lidar_topic, LaserScan, self.lidar_callback)
        self.drive_pub = Publisher(drive_topic, AckermannDriveStamped, queue_size=5)

        self.d = 0.0
        self.window_size = 1
        self.theta = math.pi / 4

        self.kp, self.ki, self.kd = params
        self.prev_error = 0.0
        self.error = 0.0
        self.square_error = 0.0
        self.integral = 0.0

        self.min_angle = 45
        self.max_angle = 135
        self.servo_offset = 0.0
        self.angle = 90.0
        self.velocity = 1.0
        self.freq = 300


        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = Time.now()
        self.drive_msg.header.frame_id = 'laser'
        self.drive_msg.drive.steering_angle = 90
        self.drive_msg.drive.speed = 0.0
        self.speed_per_angle = angle_speeds

    def set_params(self, params):
        self.kp, self.ki, self.kd = params

    def raw_distances(self, data):
        front_idx = 269

        idx_b_r = 179
        idx_b_l = 359
        idx_a_r = int(idx_b_r + round(math.degrees(self.theta), 0))
        idx_a_l = int(idx_b_l - round(math.degrees(self.theta), 0))

        a_l = np.nanmean(data.ranges[idx_a_l - self.window_size: idx_a_l + self.window_size + 1])
        b_l = np.nanmean(data.ranges[idx_b_l - self.window_size: idx_b_l + self.window_size + 1])
        a_r = np.nanmean(data.ranges[idx_a_r - self.window_size: idx_a_r + self.window_size + 1])
        b_r = np.nanmean(data.ranges[idx_b_r - self.window_size: idx_b_r + self.window_size + 1])
        a_l = a_l if a_l < 15 else 15
        b_l = b_l if b_l < 15 else 15
        a_r = a_r if a_r < 15 else 15
        b_r = b_r if b_r < 15 else 15
        return a_l, b_l, a_r, b_r

    def distance_side(self, a, b):
        y = a * math.cos(self.theta) - b
        x = a * math.sin(self.theta)
        alpha = math.atan2(y, x)
        dt = b * math.cos(alpha)
        l = 1. / self.freq * self.drive_msg.drive.speed / 10
        dt1 = dt + l * math.sin(alpha)
        return dt1

    def select_val(self, angle):
        abs_angle = abs(angle)
        if abs_angle > math.radians(20):
            velocity = self.speed_per_angle[20]
        elif abs_angle > math.radians(10):
            velocity = self.speed_per_angle[10]
        else:
            velocity = self.speed_per_angle[0]
        return velocity

    def limit_angle(self, angle):
        angle = angle + self.servo_offset
        if angle > self.max_angle + self.servo_offset:
            angle = self.max_angle + self.servo_offset
        elif angle < self.min_angle + self.servo_offset:
            angle = self.min_angle + self.servo_offset
        return angle

    def pid_control(self, error, prev_error):
        self.integral += error
        deriv = (error - prev_error) / self.freq
        angle = self.kp * error + self.kd * deriv + self.ki * self.integral
        angle = self.limit_angle(angle)
        velocity = self.select_val(angle)
        return angle, velocity

    def lidar_callback(self, data):
        a_l, b_l, a_r, b_r = self.raw_distances(data)
        dl = self.distance_side(a_l, b_l)
        dr = self.distance_side(a_r, b_r)
        self.prev_error = self.error
        self.error = (dl - dr) - self.d
        self.square_error += self.error**2
        self.angle, self.velocity = self.pid_control(self.error, self.prev_error)

        self.drive_msg.drive.steering_angle = self.angle
        self.drive_msg.drive.speed = self.velocity


def main(params=None, speeds=None):
    rospy.init_node('wallfollow', anonymous=True)
    if speeds is not None:
        wf = WallFollow(angle_speeds=speeds)
    else:
        wf = WallFollow()
    if params is not None:
        wf.set_params(params)

    rate = rospy.Rate(wf.freq)
    while not rospy.is_shutdown():
        wf.drive_pub.publish(wf.drive_msg)
        rate.sleep()
    return wf.square_error


if __name__ == '__main__':
    main()