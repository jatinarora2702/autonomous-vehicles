import os, sys
from os.path import dirname, abspath

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
import time

topic_prefix = "/pacmod/as_rx/"
pub_cmd_topic = {
    "brake": (topic_prefix + "brake_cmd", PacmodCmd),
    "accel": (topic_prefix + "accel_cmd", PacmodCmd),
    "turn": (topic_prefix + "turn_cmd", PacmodCmd),
    "steer": (topic_prefix + "steer_cmd", PositionWithSpeed),
    "enable": (topic_prefix + "enable", Bool),
}

pub_enable = rospy.Publisher(*pub_cmd_topic["enable"], queue_size=1)
pub_brake = rospy.Publisher(*pub_cmd_topic["brake"], queue_size=1)
pub_accel = rospy.Publisher(*pub_cmd_topic["accel"], queue_size=1)
pub_turn = rospy.Publisher(*pub_cmd_topic["turn"], queue_size=1)
pub_steer = rospy.Publisher(*pub_cmd_topic["steer"], queue_size=1)

rospy.init_node("control_car_non_PID", anonymous=True)
pub_enable.publish(True)

brake_time = 1
signal_time = 2
steer_time = 0.5
accel_time = 2.0

while not rospy.is_shutdown():
    try:
        key = raw_input(
            "Please enter command \n" +  
            "b: brake,    j: left signal,  k: right signal, \n" + 
            "a: left steer,  d: right steer,  w: accel, q: quit \n"
        )
        if key == "b":
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.5, enable=True)
            rospy.loginfo("Sent brake command")
            t0 = time.time()
            time.sleep(brake_time)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            rospy.loginfo("Cancelled brake command, %.3f seconds" % (time.time() - t0))

        elif key in ("j", "k"):
            pub_enable.publish(True)
            sig_dir, ui16_cmd = {
                "j": ("left", PacmodCmd.TURN_LEFT),
                "k": ("right", PacmodCmd.TURN_RIGHT),
            }[key]
            pub_turn.publish(ui16_cmd=ui16_cmd, enable=True)
            rospy.loginfo("Sent turn " + sig_dir + " command")
            t0 = time.time()
            time.sleep(signal_time)
            pub_turn.publish(ui16_cmd=PacmodCmd.TURN_NONE, enable=True)
            rospy.loginfo(
                "Cancelled turn %s command, %.3f seconds" % (sig_dir, time.time() - t0)
            )

        elif key in ("a", "d"):
            pub_enable.publish(True)
            steer_dir, th, w = {
                "a": ("left", -0.2, +0.1), 
                "d": ("right", +0.2, +0.1)
            }[key]
            pub_steer.publish(angular_position=th, angular_velocity_limit=w)
            rospy.loginfo("Sent steer " + steer_dir + " command")
            t0 = time.time()
            time.sleep(steer_time)
            rospy.loginfo(
                "Ended steer %s command, %.3f seconds" % (steer_dir, time.time() - t0)
            )

        elif key == "w":
            confirm = raw_input("Do you really want to accelerate?(y/n)")
            if not (confirm == "y"):
                continue
            pub_enable.publish(True)
            pub_accel.publish(f64_cmd=0.5, enable=True)
            rospy.loginfo("Sent accelerate command")
            t0 = time.time()
            time.sleep(accel_time)
            pub_accel.publish(f64_cmd=0.0, enable=True)
            rospy.loginfo("Cancelled accelerate command, %.3f seconds" % (time.time() - t0))

        elif key == "q":
            rospy.loginfo("quit, good bye!")
            pub_enable.publish(False)
            exit(0)
            
    except KeyboardInterrupt:
        pub_enable.publish(False)
        exit(1)


