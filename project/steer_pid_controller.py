import os
import sys

import roslib
import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from imutils import paths
import numpy as np
import imutils
from imutils.video import VideoStream
from imutils.video import FPS
import argparse


steer_pub = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
accel_pub = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
brake_pub = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
gear_pub = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)


steer_cmd = PositionWithSpeed()
accel_cmd = PacmodCmd()
brake_cmd = PacmodCmd()
gear_cmd = PacmodCmd()

class steer_pid_controller:
    def __init__(
        self,
        desired_x,
        desired_y,
        p=0.2,
        i=0.0,
        d=0.04,
        wg=20.0,
        avl=3.5,
        max_steering_angle=3.5,
        min_steering_angle=-3.5,
        sp_p=0.03,
        sp_d=0.01,
        sp_i=0.01,
        sp_wg=0.70,
        max_accel=0.35,
    ):
        #Steering Control Parameters
        self.kp = p
        self.ki = i
        self.kd = d
        self.windup_guard = wg
        self.prev_error = 0.0
        self.prev_time = time.time()

        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0

        self.max_steering_angle = max_steering_angle
        self.min_steering_angle = min_steering_angle

        self.angular_velocity_limit = avl
        self.desired_x = desired_x

        global steer_pub
        global steer_cmd

        #Speed Control Parameters
        self.sp_kp = sp_p
        self.sp_kd = sp_d
        self.sp_ki = sp_i
        self.sp_windup_guard = sp_wg

        self.sp_Pterm = 0.0
        self.sp_Iterm = 0.0
        self.sp_Dterm = 0.0

        self.sp_prev_error = 0.0
        self.sp_prev_time = time.time()
        self.desired_y = desired_y
        self.max_accel = max_accel
        global accel_cmd
        global brake_cmd
        global accel_pub
        global brake_pub
        global gear_cmd
        global gear_pub

        accel_cmd.enable = False
        accel_cmd.clear = True
        accel_cmd.ignore = True
        brake_cmd.enable = False
        brake_cmd.clear = True
        brake_cmd.ignore = True

    def steer_control(self, curr_x):
        global steer_cmd
        steer_flag = True
        if steer_flag:
            current_time = time.time()
            delta_time = current_time - self.prev_time

            current_error = self.desired_x - curr_x
            delta_error = current_error - self.prev_error
            error_dot = delta_error / delta_time

            self.pterm = current_error
            self.dterm = error_dot
            self.iterm += current_error * delta_time

            if self.iterm > self.windup_guard:
                self.iterm = self.windup_guard
            if self.iterm < -self.windup_guard:
                self.iterm = -self.windup_guard

            self.prev_time = current_time
            self.prev_error = current_error

            output = (
                self.kp * self.pterm
                + self.kd * self.dterm
                + self.ki * self.iterm
            )

            output = max(min(output, self.max_steering_angle),
                         self.min_steering_angle)
            output = np.pi * output

            steer_cmd.angular_position = output
            steer_cmd.angular_velocity_limit = (np.pi / 2) / 2
            steer_pub.publish(steer_cmd)

    def speed_control(self, curr_y):
        global accel_cmd
        global brake_cmd
        global accel_pub
        global brake_pub
        global gear_cmd
        global gear_pub

        accel_cmd.enable = True
        accel_cmd.clear = False
        accel_cmd.ignore = False

        brake_cmd.enable = True
        brake_cmd.clear = False
        brake_cmd.ignore = False

        current_time = time.time()
        delta_time = current_time - self.prev_time

        current_error = self.desired_y - curr_y
        # current_error *= -1
        delta_error = current_error - self.sp_prev_error
        error_dot = delta_error / delta_time
        
        self.sp_Pterm = current_error
        self.sp_Dterm = error_dot
        self.sp_Iterm += current_error * delta_time

        if self.sp_Iterm > self.sp_windup_guard:
            self.sp_Iterm = self.sp_windup_guard
        if self.sp_Iterm < -self.sp_windup_guard:
            self.sp_Iterm = -self.sp_windup_guard

        self.prev_time = current_time
        self.sp_prev_error = current_error

        # For debugging
        # print("kp:", self.sp_kp, "pterm", self.sp_Pterm, "2nd:", (self.sp_kd * self.sp_Dterm), "3rd: ", self.sp_ki * self.sp_Iterm)
        output = (
            self.sp_kp * self.sp_Pterm
            + self.sp_kd * self.sp_Dterm
            + self.sp_ki * self.sp_Iterm
        )

        Th = 7

        #Braking
        if current_error <= Th and current_error >= -Th:
            brake_cmd.f64_cmd = 0.7
            accel_cmd.f64_cmd = 0
            print("Braking")

        elif current_error > Th:
            brake_cmd.f64_cmd = 0.0
            gear_cmd.ui16_cmd = 3
            gear_pub.publish(gear_cmd)
            print("Accelerating")

        elif current_error < -Th:
            brake_cmd.f64_cmd = 0.0
            gear_cmd.ui16_cmd = 1
            output = abs(output)
            gear_pub.publish(gear_cmd)
            print("Reverse")

        if output > self.max_accel:
            output = self.max_accel
        elif output < 0.2:
            output = 0.2

        accel_cmd.f64_cmd = output

        print("Final Acceleration: ", accel_cmd.f64_cmd)
        print("Final Brake: ", brake_cmd.f64_cmd)
        
        accel_pub.publish(accel_cmd)
        brake_pub.publish(brake_cmd)
