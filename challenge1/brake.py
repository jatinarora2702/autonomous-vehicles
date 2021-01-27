import os, sys
from os.path import dirname, abspath

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from imutils import paths
import numpy as np
import imutils

brake_time = 1
signal_time = 2
steer_time = 0.5
accel_time = 2.0


class PedestrianDetector:
    def __init__(self):
        # initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def pedestrian_exists(self, image):
        image = imutils.resize(image, width=min(400, image.shape[1]))
        (rects, weights) = self.hog.detectMultiScale(
            image, winStride=(4, 4), padding=(8, 8), scale=1.05
        )
        threshold = 0.1
        for weight in weights:
            if weight > threshold:
                return True
        return False


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = PedestrianDetector()
        self.image_sub = rospy.Subscriber(
            "/mako_1/mako_1/image_raw", Image, self.callback
        )

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            person_exists = self.detector.pedestrian_exists(cv_image)
            if person_exists:
                rospy.loginfo("Pedestrian in the frame, braking ...")
                pub_enable.publish(True)
                pub_brake.publish(f64_cmd=0.5, enable=True)
                rospy.loginfo("sent brake command")
                t0 = time.time()
                time.sleep(brake_time)
                pub_brake.publish(f64_cmd=0.0, enable=True)
                rospy.loginfo("cancelled brake command, %.3f sec" % (time.time() - t0))
            else:
                print("No pedestrian, safe to drive ...")
        except CvBridgeError as e:
            print(e)


rospy.init_node("image_converter", anonymous=True)

topic_prefix = "/pacmod/as_rx/"
pub_cmd_topic = {
    "brake": (topic_prefix + "brake_cmd", PacmodCmd),
    "accel": (topic_prefix + "accel_cmd", PacmodCmd),
    "turn": (topic_prefix + "turn_cmd", PacmodCmd),
    "steer": (topic_prefix + "steer_cmd", PositionWithSpeed),
    "enable": (topic_prefix + "enable", Bool),
}

pub_enable = rospy.Publisher(*pub_cmd_topic["enable"], queue_size=1)
pub_brake = rospy.Publisher(
    *pub_cmd_topic["brake"], queue_size=1
)  # queue_size not handled
pub_accel = rospy.Publisher(*pub_cmd_topic["accel"], queue_size=1)
pub_turn = rospy.Publisher(*pub_cmd_topic["turn"], queue_size=1)
pub_steer = rospy.Publisher(*pub_cmd_topic["steer"], queue_size=1)

ic = image_converter()
pub_enable.publish(True)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
