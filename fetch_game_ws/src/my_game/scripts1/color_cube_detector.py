#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class ColorCubeDetector:
    def __init__(self):
        rospy.init_node('color_cube_detector')
        self.bridge = cv_bridge.CvBridge()

        rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_callback)
        rospy.loginfo("Color Cube Detector Initialized")
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rouge
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        # Bleu
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Détection rouge
        self.detect_and_draw(frame, mask_red, "Red", (0, 0, 255))

        # Détection bleu
        self.detect_and_draw(frame, mask_blue, "Blue", (255, 0, 0))

        cv2.imshow("Cube Detector", frame)
        cv2.waitKey(1)

    def detect_and_draw(self, frame, mask, label, color_bgr):
        # Correction ici pour OpenCV 2/3
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(frame, "{}: ({}, {})".format(label, cx, cy), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

if __name__ == "__main__":
    try:
        ColorCubeDetector()
    except rospy.ROSInterruptException:
        pass
