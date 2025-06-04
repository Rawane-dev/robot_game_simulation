#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class CubePicker:
    def __init__(self):
        rospy.init_node('cube_picker')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        self.pc_sub = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        try:
            self.head = moveit_commander.MoveGroupCommander("head")
        except RuntimeError:
            self.head = None
            rospy.logwarn("Head group not found. Skipping head movement.")

        self.latest_cloud = None
        self.target_pixel = None

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

     # Yellow range (replace blue)
     lower_yellow = np.array([20, 100, 100])
     upper_yellow = np.array([30, 255, 255])
     mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

     # Combine red and yellow
     combined_mask = mask_red + mask_yellow

     contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

     if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:
            x, y, w, h = cv2.boundingRect(largest)
            cx, cy = x + w // 2, y + h // 2
            self.target_pixel = (cx, cy)

            # Determine cube color
            cube_hue = hsv[cy, cx][0]
            if 20 <= cube_hue <= 30:
                self.detected_color = "yellow"
            elif cube_hue <= 10 or cube_hue >= 160:
                self.detected_color = "red"
            else:
                self.detected_color = "unknown"

            rospy.loginfo("Detected a %s cube at pixel (%d, %d)", self.detected_color, cx, cy)

            # Draw a rectangle on the original image
            color = (0, 255, 255) if self.detected_color == "yellow" else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)


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

    def pick_cube(self):
        """Try to pick up a cube with improved detection and error handling"""
        # Wait for cube detection
        detection_timeout = 5.0  # seconds to wait for target detection
        start_time = rospy.Time.now().to_sec()
        
        # Wait for target detection
        while self.target_pixel is None:
            if rospy.Time.now().to_sec() - start_time > detection_timeout:
                rospy.logwarn("No cube detected within timeout period.")
                return False
            rospy.sleep(0.5)
            
        rospy.loginfo("Cube detected! Color: {self.detected_color}")
        rospy.sleep(1)
        
        point = self.pixel_to_3d_point(self.target_pixel)
        if point is None or np.isnan(point[0]):
            rospy.logwarn("Invalid 3D point from point cloud.")
            return False

        transformed = self.transform_point(point)
        if transformed is None:
            rospy.logwarn("Failed to transform point to base_link frame.")
            return False

        x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z

        # Validate if the point is within reasonable bounds
        if x < 0.3 or x > 1.2:
            rospy.logwarn("Cube position out of reasonable range (x={x:.2f}). Skipping...")
            return False
        
        # Begin grasp sequence
        self.open_gripper()
        rospy.sleep(1)

        # First move to a position above the cube
        above_pose = self.create_pose(0.5, y, z + 0.07)
        if not self.move_to_pose(above_pose):
            rospy.logwarn("Failed to move to position above cube.")
            return False
            
        rospy.loginfo("Approached cube from above.")
        rospy.sleep(1)

        # Then move to the actual grasp position
        grasp_pose = self.create_pose(x - 0.12, y, 0.8)
        if not self.move_to_pose(grasp_pose):
            rospy.logwarn("Failed to move to grasping position.")
            return False
            
        rospy.loginfo("At grasp position.")
        rospy.sleep(1)
        
        # Close the gripper to grab the cube
        self.close_gripper()
        rospy.sleep(1)

        # Move back with the cube
        retreat_pose = self.create_pose(x - 0.2, y, z+0.05)
        if not self.move_to_pose(retreat_pose):
            rospy.logwarn("Failed to retreat with cube. Cube may not be securely grasped.")
            return False
            
        rospy.loginfo("Successfully grasped and pulled cube back.")
        
        # Reset target pixel after successful grasp
        self.target_pixel = None
        return True

    def run(self):
        rospy.sleep(3)
        while not rospy.is_shutdown():
            self.pick_cube()
            rospy.sleep(5)


if __name__ == '__main__':
    try:
        picker = CubePicker()
        picker.run()
    except rospy.ROSInterruptException:
        pass
