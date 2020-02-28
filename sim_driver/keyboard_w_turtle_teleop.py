#!/usr/bin/env python

# KEYBOARD CONTROL RACECAR using turtle_teleop_key

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import time

class KeyBoard:
    def __init__(self, pub_opt, sub_opt):
	    #TODO: initiate AckermannDriveStamped message that will be published
        self.ack_msg = AckermannDriveStamped()
        self.ack_msg.header.stamp = rospy.Time.now()
        self.ack_msg.header.frame_id = ''
        self.ack_msg.drive.steering_angle = 0.0
        self.ack_msg.drive.speed = 0.0
        self.pub = rospy.Publisher(pub_opt['topic_name'], AckermannDriveStamped, queue_size=10) # Publish to /vesc/high_level/ackermann_cmd_mux/input/nav_0
        self.sub = rospy.Subscriber(sub_opt['topic_name'], Twist, self.callback) # Subscribe to /keyboard from teleop_key
        self.curr_angle = 0.0
        self.speed = 0.0
        self.speed_delta = 3.0
        self.turn_angle = 0.2 #radians
        self.turn_speed = 0.4 #radians / sec
        self.last_key_press_time = time.time()
    def callback(self, msg):
	    # TODO:
	    # - read Twist msg to detect direction command (up, down, left, right)
	    # - then set speed and velocity properly in ack_msg
        # - also store trigger time
        self.ack_msg.header.stamp = rospy.Time.now()# Trigger time
        self.last_key_press_time = time.time()
        if msg.linear.x < 0 and msg.angular.z == 0: # KEY_DOWN
            self.ack_msg.drive.steering_angle = 0.0
            self.ack_msg.drive.speed = self.speed - self.speed_delta
        elif msg.linear.x > 0 and msg.angular.z == 0:
            self.ack_msg.drive.steering_angle = 0.0
            self.ack_msg.drive.speed = self.speed + self.speed_delta
        elif msg.linear.x == 0 and msg.angular.z > 0:
            self.ack_msg.drive.steering_angle = self.curr_angle + self.turn_angle
            self.ack_msg.drive.steering_angle_velocity = self.turn_speed
            self.ack_msg.drive.speed = self.speed
        elif msg.linear.x == 0 and msg.angular.z < 0:
            self.ack_msg.drive.steering_angle = self.curr_angle - self.turn_angle
            self.ack_msg.drive.steering_angle_velocity = self.turn_speed
            self.ack_msg.drive.speed = self.speed
        else:
            self.ack_msg.drive.speed = 0.0

        
    def process_keyboard(self):
	    # TODO: calculate `timeout` - the duration between current time (now) and
        # the last keypress (the last trigger time of callback() above stored in ack_msg).
        # If it is longer than a threshold (0.1 sec), set speed and direction to 0 to stop the car
        timeout = time.time() - self.last_key_press_time
        if timeout > 0.1: # latest keypress is older than 0.1s ago
            self.ack_msg.drive.speed = 0.0

        rospy.loginfo("ang: %g, vel: %g", self.ack_msg.drive.steering_angle, self.ack_msg.drive.speed)

        self.pub.publish(self.ack_msg)


if __name__=='__main__':
    rospy.init_node("keyboard_node")
    kb = KeyBoard({'topic_name': '/drive'}, {'topic_name': 'my_teleop'})
    rospy.loginfo("Initialized: keyboard_node")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        kb.process_keyboard()
        rate.sleep()
