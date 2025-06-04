#!/usr/bin/env python
# -*- coding: utf-8


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(msg):
    rospy.loginfo("Image reçue")
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("Erreur conversion : %s", e)
        return

    cv2.imwrite("/tmp/image_camera.png", img)
    rospy.loginfo("Image enregistrée dans /tmp/image_camera.png")
    cv2.imshow("Image Caméra", img)
    cv2.waitKey(1)

def main():
    rospy.init_node("test_camera_image")
    rospy.Subscriber("/head_camera/rgb/image_raw", Image, callback)
    rospy.loginfo("Attente d'images...")
    rospy.spin()

if __name__ == '__main__':
    main()


