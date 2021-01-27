import os
import sys

import dlib
from steer_pid_controller import steer_pid_controller
import roslib
import rospy
from std_msgs.msg import Bool, String, Float32, Float64
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
from pynput.keyboard import Key, Listener, KeyCode

steer_pub = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
gear_pub = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
accel_pub = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
brake_pub = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)

enabled = False
accel_flag = False
gear_cmd = PacmodCmd()
accel_cmd = PacmodCmd()
brake_cmd = PacmodCmd()
steer_cmd = PositionWithSpeed()

rospy.init_node("object_tracker", anonymous=True)

def on_press(key):
    global enabled
    global gear_cmd
    global accel_cmd
    global brake_cmd
    global accel_flag

    if key == KeyCode.from_char("p"):
        print("TRACKER ENGAGED")
        enabled = True
        accel_cmd.enable = True
        accel_cmd.clear = False
        accel_cmd.ignore = False
        brake_cmd.enable = True
        brake_cmd.clear = False
        brake_cmd.ignore = False
        accel_cmd.f64_cmd = 0.0
        brake_cmd.f64_cmd = 0.0
        gear_cmd.ui16_cmd = 3

    enable_pub.publish(Bool(enabled))
    gear_pub.publish(gear_cmd)
    brake_pub.publish(brake_cmd)



enable_pub.publish(Bool(enabled))
listener = Listener(on_press=on_press)
listener.start()



class object_tracker:
    def __init__(self):
        self.bridge = CvBridge()
        # subscribe to the camera feed
        self.image_sub = rospy.Subscriber(
            "/mako_1/mako_1/image_raw", Image, self.callback
        )

        self.tracker = dlib.correlation_tracker()
        self.initBB = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = imutils.resize(cv_image, width=500)
            (H, W) = frame.shape[:2]

            if self.initBB is not None:
                self.tracker.update(frame)
                bbox = self.tracker.get_position()
                x1, y1 = int(bbox.left()), int(bbox.top())
                x2, y2 = int(bbox.right()), int(bbox.bottom())
                cv2.rectangle(frame, (x1, y1), (x2, y2),
                              (0, 255, 0), 2)
                self.steer.speed_control(y2-y1)
                self.steer.steer_control((x1 + x2) / 2)
                self.fps.update()
                self.fps.stop()
                info = [
                    ("Height", "{:.2f}".format((y2 - y1))),
                    ("FPS", "{:.2f}".format(self.fps.fps())),
                ]
                for (i, (k, v)) in enumerate(info):
                    text = "{}: {}".format(k, v)
                    cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("ROS Camera Feed", frame)
            key = cv2.waitKey(1) & 0xFF

            # if the 's' key is selected, we are going to "select" a bounding
            # box to track
            if key == ord("s"):
                self.initBB = cv2.selectROI(
                    "ROS Camera Feed", frame, fromCenter=False, showCrosshair=True)
                # self.tracker.init(frame, self.initBB)
                # tracker.init(frame, initBB)
                x, y, w, h = self.initBB
                points = [x, y, (x + w), (y + h)]
                self.tracker.start_track(frame, dlib.rectangle(*points))
                self.fps = FPS().start()
                desired_x = x + (w / 2)
                desired_y = h
                self.steer = steer_pid_controller(desired_x, desired_y)

            # if the `q` key was pressed, break from the loop
            elif key == ord("q"):
                sys.exit(0)

        except CvBridgeError as e:
            print(e)

ic = object_tracker()

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Shutting down")
