#!/usr/bin/env python
# -- coding: utf-8 --

import sys
import rospy
import moveit_commander
import numpy as np
import math
import cv2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_ros
import os
from gazebo_msgs.srv import SpawnModel
from test import FetchRotator
from arm import open_fetch_arm_topdown
class CubePicker:
    def __init__(self):
        rospy.init_node('cube_picker', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        self.pc_sub = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.spawn_counter = 14
        self.rotator = FetchRotator()
        try:
            self.head = moveit_commander.MoveGroupCommander("head")
        except RuntimeError:
            self.head = None
            rospy.logwarn("Head group not found. Skipping head movement.")

        self.latest_cloud = None
        self.target_pixel = None
        #self.spawn_jeton("player2")

    def pc_callback(self, msg):
        self.latest_cloud = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red range (two segments)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        # Blue range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)


        # Combine both masks (if detecting any cube)
        combined_mask = mask_red + mask_blue

        _, contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                x, y, w, h = cv2.boundingRect(largest)
                cx, cy = x + w // 2, y + h // 2
                self.target_pixel = (cx, cy)

                # Determine cube color
                cube_hue = hsv[cy, cx][0]
                if 100 <= cube_hue <= 140:
                    self.detected_color = "blue"
                elif cube_hue <= 10 or cube_hue >= 160:
                    self.detected_color = "red"
                else:
                    self.detected_color = "unknown"

                rospy.loginfo("Detected a %s cube at pixel (%d, %d)", self.detected_color, cx, cy)

                # Draw a rectangle on the original image
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0) if self.detected_color == "blue" else (0, 0, 255), 2)

        # Optional visualization
        #cv2.imshow("Camera View", frame)
        #cv2.waitKey(1)


    def pixel_to_3d_point(self, pixel):
        if self.latest_cloud is None:
            return None
        u, v = pixel
        gen = pc2.read_points(self.latest_cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=[(u, v)])
        return next(gen, None)

    def transform_point(self, point):
        pose = PoseStamped()
        pose.header.frame_id = self.latest_cloud.header.frame_id
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform("base_link", pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            rospy.loginfo("3D point: (%.3f, %.3f, %.3f)", transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z)
            return transformed_pose
        except Exception as e:
            rospy.logerr("TF transform error: %s", str(e))
            return None

    def open_gripper(self):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 0.049
        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper opened.")

    def close_gripper(self):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 0.01
        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper closed.")

    def lower_head(self):
        if self.head is not None:
            joint_goal = self.head.get_current_joint_values()
            joint_goal[0] = -0.6
            self.head.go(joint_goal, wait=True)
            rospy.loginfo("Head lowered.")
        else:
            rospy.loginfo("No head movement.")

    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(0.0, 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def move_to_pose(self, pose):
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def spawn_jeton(self, player):
        if player == "player1":
            model_dir = 'number_1'  # red cube
        elif player == "player2":
            model_dir = 'number2'    # blue cube
        else:
            rospy.logerr("Unknown player: {}. Must be 'player1' or 'player2'.".format(player))
            return

        model_path = os.path.join(os.environ['HOME'], '.gazebo', 'models', 'ar_tag_alvar', model_dir, 'model.sdf')
        with open(model_path, 'r') as f:
            model_xml = f.read()

        x = 1.18387239174
        y = -0.300185412203
        z = 0.67723610508
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0.012233575938
        pose.orientation.y = 0.707001000096
        pose.orientation.z = -0.0122334687864
        pose.orientation.w = 0.707000896551

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            model_name = "new_cube_{}".format(self.spawn_counter)
            spawn_model(model_name=model_name, model_xml=model_xml, robot_namespace='', initial_pose=pose, reference_frame='world')
            self.spawn_position = (x, y, z)
            rospy.loginfo("Spawned %s for %s at x=%.2f, y=%.2f, z=%.2f", model_name, player, x, y, z)
        except Exception as e:
            rospy.logerr("Failed to spawn: %s", str(e))


    def pick_cube(self):
        if self.target_pixel is None:
            rospy.loginfo("No target detected.")
            return

        rospy.sleep(1)
        point = self.pixel_to_3d_point(self.target_pixel)
        if point is None or np.isnan(point[0]):
            rospy.logwarn("Invalid 3D point.")
            return

        transformed = self.transform_point(point)
        if transformed is None:
            return

        x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z

        if x < 0.3 or x > 1.2:
            rospy.logwarn("Cube too far or too close (x=%.2f). Skipping...", x)
            return

        self.open_gripper()
        rospy.sleep(1)

        above_pose = self.create_pose(0.5, y, z + 0.07)
        if self.move_to_pose(above_pose):
            rospy.loginfo("Approached cube.")
            rospy.sleep(1)

            grasp_pose = self.create_pose(x - 0.12, y , 0.67)
            if self.move_to_pose(grasp_pose):
                rospy.loginfo("At grasp position.")
                rospy.sleep(1)
                self.close_gripper()
                rospy.sleep(1)

                retreat_pose = self.create_pose(x - 0.2, y, z+0.05)
                self.move_to_pose(retreat_pose)
                rospy.loginfo("Pulled cube back.")


            else:
                rospy.logwarn("Failed to move to grasping position.")

    def rotate_180(self):
        rospy.loginfo("Rotating 180° using cmd_vel...")
        twist = Twist()
	twist.linear.x = 0.0
	twist.linear.y = 0.0
	twist.linear.z = 0.0
	twist.angular.x = 0.0
	twist.angular.y = 0.0
	twist.angular.z = 0.5  # Rotation sur place
        duration = 8.0

        rate = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_vel.publish(twist)
            rate.sleep()

        twist.angular.z = 0
        self.cmd_vel.publish(twist)
        rospy.loginfo("Rotation complete.")

    def move_to_connect4_column(self, column_index):
        column_positions = [
            (0.64, -0.13, 0.59),  # Column 0
            (0.64, -0.13, 0.59),  # Column 1
            (0.64, -0.09, 0.59),  # Column 2
            (0.64, 0.02, 0.59),   # Column 3
            (0.64, 0.07, 0.59),   # Column 4
            (0.66, 0.23, 0.59),   # Column 5
            (0.66, 0.33, 0.59)    # Column 6
        ]

        if not (0 <= column_index < len(column_positions)):
            rospy.logerr("Invalid column index: {}. Must be between 0 and 6.".format(column_index))
            return

        x, y, z = column_positions[column_index]
        rospy.loginfo("Moving to gap {}".format(column_index + 1))

        pose_target = PoseStamped()
        pose_target.header.frame_id = "base_link"
        pose_target.pose.orientation.w = 1.0
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = z

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose_target)

        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        if success:
            rospy.loginfo("Moved to gap {}".format(column_index + 1))
            self.open_gripper()
        else:
            rospy.logwarn("Failed to move to gap {}".format(column_index + 1))

    def run(self, col, player):
        rospy.sleep(3)
        self.pick_cube()
        rospy.sleep(5)
        rospy.loginfo("Attempting to rotate to face the board...")
        self.rotator.rotate_to_face_point(-2.55, 0.09)
        rospy.sleep(2)
        self.move_to_connect4_column(col)
        rospy.sleep(2)
        rospy.loginfo("Moving arm to top-down position before rotating back...")
        open_fetch_arm_topdown()
  


if __name__ == '__main__':

    try:
        picker = CubePicker()
        #picker.run(3,"player2")
	#rospy.sleep(2)
        picker.run(2,"player1")
    except rospy.ROSInterruptException:
        pass
