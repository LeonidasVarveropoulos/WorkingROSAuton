#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import sys
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np
import roslib

class TurretRotation:
    def __init__(self):
        rospy.init_node('turret_rotation_tuner')
        self.cmd_vel_pub = rospy.Publisher("turret/cmd_vel", Float32, queue_size=50)

        self.update_rate = 20

        self.pid = PID(self.update_rate, 5.0, -5.0)
        # This is the feedback and setpoint from the vision processing node (pixels)
        self.feedback = 0.0
        self.setpoint = 0.0

        # Thsi is the control output of the pid that will be sent to the roboRio (velocity)
        self.control = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.kf = 0.0

    def feed_callback(self, msg):
        # Feedback from vision targeting
        self.feedback = msg.data
        print("Got Data")

    def set_callback(self, msg):
        # Setpoint from vision targeting
        self.setpoint = msg.data
    
    def kp_callback(self, msg):
        # Setpoint from vision targeting
        self.kp = msg.data
    
    def ki_callback(self, msg):
        # Setpoint from vision targeting
        self.ki = msg.data
    
    def kd_callback(self, msg):
        # Setpoint from vision targeting
        self.kd = msg.data
    
    def kf_callback(self, msg):
        # Setpoint from vision targeting
        self.kf = msg.data
    
    def main(self):

        feed_sub = rospy.Subscriber('turret/feedback', Float32, self.feed_callback)
        set_sub = rospy.Subscriber('turret/setpoint', Float32, self.set_callback)

        kp_sub = rospy.Subscriber('turret/kp', Float32, self.kp_callback)
        ki_sub = rospy.Subscriber('turret/ki', Float32, self.ki_callback)
        kd_sub = rospy.Subscriber('turret/kd', Float32, self.kd_callback)
        kf_sub = rospy.Subscriber('turret/kf', Float32, self.kf_callback)

        r = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self.pid.update_pid(self.setpoint, self.feedback, self.kp, self.ki, self.kd, self.kf)

            self.control = self.pid.get_control()

            # Publishes the rpm to ros for the proxy to send to the roboRio
            data = Float32()
            data.data = self.control
            self.cmd_vel_pub.publish(data)

            print(self.kp,self.ki,self.kd,self.kf)
            
            r.sleep()

class PID:
    def __init__(self, update_rate, control_max, control_min):
        self.control = 0.0
        self.error_sum = 0.0
        self.update_rate = update_rate
        self.control_max = control_max 
        self.control_min = control_min
        self.prev_error = 0.0
    
    def reset(self):
        self.control = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update_pid(self, setpoint, feedback, kp, ki, kd, kf):
        # Calculate error
        error = (setpoint) - feedback
        
        # Calculate sum of error over time
        self.error_sum += (error/self.update_rate) # Not sure if this should be here

        if ki != 0:
            error_sum_max = self.control_max / ki
            error_sum_min = self.control_min / ki
            if self.error_sum > error_sum_max:
                self.error_sum = error_sum_max
            elif self.error_sum < error_sum_min:
                self.error_sum = error_sum_min
        
        self.control = (setpoint * kf) + (kp * error) + (ki *self.error_sum) + (kd * ((error - self.prev_error) * self.update_rate))

        if self.control > self.control_max:
            self.control = self.control_max
        elif self.control < self.control_min:
            self.control = self.control_min
        self.prev_error = error

    def get_control(self):
        return self.control

tr = TurretRotation()
tr.main()

