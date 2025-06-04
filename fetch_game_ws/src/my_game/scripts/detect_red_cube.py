#!/usr/bin/env python
# -*- coding: utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, Point  # Import des types géométriques
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class RedCubeDetector:
    def __init__(self):
        rospy.init_node('red_cube_detector')
        self.bridge = CvBridge()

        self.u = None
        self.v = None
        self.new_detection = False
        self.pub = rospy.Publisher('/red_cub_position', PointStamped, queue_size=10)

        rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2, self.pointcloud_callback)

        rospy.loginfo("Détecteur de cube rouge lancé.")
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Erreur conversion image : %s", str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Masque pour rouge (deux intervalles pour le rouge en HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.u = cX
                self.v = cY
                self.new_detection = True
                rospy.loginfo("Cube rouge détecté à l'image : (%d, %d)", cX, cY)

    def pointcloud_callback(self, msg):
       if not self.new_detection or self.u is None or self.v is None:
          return

       try:
          gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True, uvs=[[self.u, self.v]])
          point = next(gen)

          x, y, z = point
          rospy.loginfo("Coordonnées du cube rouge : x=%.2f, y=%.2f, z=%.2f", x, y, z)

          # ✅ Publier la position
          point_msg = PointStamped()
          point_msg.header.stamp = rospy.Time.now()
          point_msg.header.frame_id = msg.header.frame_id  # souvent "head_camera_rgb_optical_frame"
          point_msg.point.x = x
          point_msg.point.y = y
          point_msg.point.z = z

          self.pub.publish(point_msg)
          self.new_detection = False  # éviter de republier sans nouvelle détection

       except Exception as e:
        rospy.logwarn("Erreur lors de la lecture du point cloud : %s", str(e))



if __name__ == "__main__":
    RedCubeDetector()

