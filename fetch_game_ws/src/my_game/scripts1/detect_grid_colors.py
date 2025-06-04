#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
import os
import numpy as np
import time
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

COORDS_FILE = os.path.join(os.path.dirname(__file__), "grid_coords.yaml")
bridge = CvBridge()

def detect_color(bgr):
    b, g, r = bgr
    if r > 120 and g < 100 and b < 100:
        return 1  # Red (human)
    elif r > 150 and g > 150 and b < 120:
        return 2  # Yellow (robot)
    else:
        return 0  # Empty

def is_board_invalid(grid):
    yellow_count = sum(cell == 2 for row in grid for cell in row)
    red_count = sum(cell == 1 for row in grid for cell in row)

    if red_count > yellow_count or yellow_count - red_count > 1:
        rospy.logwarn("❌ Invalid token count: Yellow = %d, Red = %d", yellow_count, red_count)
        return True

    for col in range(7):
        found_empty = False
        for row in range(5, -1, -1):  # bottom to top
            cell = grid[row][col]
            if cell == 0:
                found_empty = True
            elif found_empty and cell != 0:
                rospy.logwarn("❌ Floating token detected at column %d, row %d", col, row)
                return True

    return False

def move_head(pan_direction):
    client = actionlib.SimpleActionClient('/head_controller/point_head', PointHeadAction)
    client.wait_for_server()

    goal = PointHeadGoal()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1.0
    goal.target.point.y = pan_direction
    goal.target.point.z = 1.2

    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 1.0

    rospy.loginfo("Sending head goal to y = %f" % pan_direction)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Head movement complete for y = %f" % pan_direction)

def shake_head():
    rospy.loginfo("🙅 Shaking head left and right...")
    directions = [-0.3, 0.3, -0.1]
    for pan in directions:
        move_head(pan)
        time.sleep(1.0)

def callback(msg):
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        with open(COORDS_FILE, 'r') as f:
            data = yaml.safe_load(f)

        coords = data.get("grid_coords", []) if isinstance(data, dict) else data
        if not coords:
            rospy.logwarn("No coordinates found in YAML file.")
            return

        grid = [[0 for _ in range(7)] for _ in range(6)]
        image_visu = image.copy()

        for idx, coord in enumerate(coords):
            if isinstance(coord, (list, tuple)) and len(coord) == 2:
                x, y = coord
                bgr = image[y, x]
                rospy.loginfo("Coord %d (%d,%d) → BGR = %s", idx, x, y, str(bgr))

                color_code = detect_color(bgr)
                row = idx // 7
                col = idx % 7
                grid[row][col] = color_code

                color_visu = (0, 255, 0) if color_code == 0 else ((0, 0, 255) if color_code == 1 else (0, 255, 255))
                cv2.circle(image_visu, (x, y), 5, color_visu, -1)
        
        rospy.loginfo("🎯 Grid State:")
        for row in grid:
            rospy.loginfo(str(row))
        
        if is_board_invalid(grid):
            rospy.logerr("🚫 Invalid board detected!")
            shake_head()
        else:
            rospy.loginfo("✅ Board is valid.")

        #cv2.imshow("Grille Puissance4", image_visu)
        #cv2.waitKey(1)

    except Exception as e:
        rospy.logerr_throttle(2, "Error: {}".format(e))

if __name__ == '__main__':
    rospy.init_node("detect_grid_colors_node")
    rospy.loginfo("📸 Subscribing to RGB camera...")
    rospy.Subscriber("/head_camera/rgb/image_raw", Image, callback)
    rospy.loginfo("⏳ Waiting for camera image...")
    rospy.spin()
    cv2.destroyAllWindows()
