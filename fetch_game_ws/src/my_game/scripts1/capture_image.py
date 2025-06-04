#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite("/tmp/image_camera.png", cv_image)
        rospy.loginfo("✅ Image capturée et enregistrée dans /tmp/image_camera.png")
    except Exception as e:
        rospy.logerr("Erreur lors de la conversion de l'image : {}".format(e))

rospy.init_node('capture_image_node')
rospy.Subscriber("/head_camera/rgb/image_raw", Image, callback)

rospy.loginfo("⏳ En attente d'une image de la caméra RGB...")
rospy.spin()

s
