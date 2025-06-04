#!/usr/bin/env python

# coding: utf-8 --

import sys
import rospy
import moveit_commander
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_ros
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CubePicker:
    def __init__(self):
        rospy.init_node('cube_picker', anonymous=True)
        self.bridge = CvBridge()

        # --- IMPORTANT FIX: Initialize attributes BEFORE Subscribers ---
        # Detection variables
        self.latest_cloud = None
        self.target_pixel = None
        self.detected_color = None
        self.selected_color = None # Initialize selected_color here to prevent AttributeError

        # Subscribe to camera feeds
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        self.pc_sub = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc_callback)

        # TF setup for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize MoveIt for arm control
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        # Initialize gripper control
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        
        # Initialize torso control using action client (more reliable)
        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        
        rospy.loginfo("Waiting for torso controller...")
        self.torso_available = self.torso_client.wait_for_server(timeout=rospy.Duration(5.0))
        if self.torso_available:
            rospy.loginfo("Torso controller connected successfully")
        else:
            rospy.logwarn("Torso controller not available")

    def pc_callback(self, msg):
        """Store the latest point cloud data"""
        self.latest_cloud = msg
        
    def image_callback(self, msg):
        """Detect green, baby blue, orange, purple, pink, brown, and mustard cubes"""
        # Ensure selected_color is set before proceeding
        if self.selected_color is None:
            return 

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for each color
        color_ranges = {
            "green": [(np.array([45, 100, 100]), np.array([75, 255, 255]))],
            "baby_blue": [(np.array([85, 50, 100]), np.array([105, 255, 255]))],
            "light_purple": [(np.array([135, 60, 130]), np.array([155, 160, 255]))],
            "purple": [(np.array([125, 100, 100]), np.array([155, 255, 255]))],
            "pink": [(np.array([160, 100, 100]), np.array([175, 255, 255]))],
            "brown": [(np.array([10, 100, 20]), np.array([20, 255, 200]))],  # Dark muted orange range
            "mustard": [(np.array([20, 150, 150]), np.array([30, 255, 200]))]  # Yellowish brown
        }

        # Only process the selected color
        if self.selected_color in color_ranges:
            for lower, upper in color_ranges[self.selected_color]:
                mask = cv2.inRange(hsv, lower, upper)
                
                # OpenCV 2.x/3.x compatibility for findContours
                try:
                    # For OpenCV 3.x and newer, findContours returns (image, contours, hierarchy)
                    _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                except ValueError:
                    # For OpenCV 2.x, findContours returns (contours, hierarchy)
                    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest) > 500:
                        x, y, w, h = cv2.boundingRect(largest)
                        cx, cy = x + w // 2, y + h // 2
                        self.target_pixel = (cx, cy)
                        self.detected_color = self.selected_color # Set detected_color to the selected one
                        rospy.loginfo("Detected a %s cube at pixel (%d, %d)", self.selected_color, cx, cy)
                        return  # Stop after first detected cube

    def pixel_to_3d_point(self, pixel):
        """Convert 2D pixel coordinates to 3D world coordinates using point cloud"""
        if self.latest_cloud is None:
            return None
        u, v = pixel
        gen = pc2.read_points(self.latest_cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=[(u, v)])
        return next(gen, None)

    def transform_point(self, point):
        """Transform 3D point from camera frame to base_link frame"""
        pose = PoseStamped()
        pose.header.frame_id = self.latest_cloud.header.frame_id
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform("base_link", pose.header.frame_id, 
                                                  rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            rospy.loginfo("3D point: (%.3f, %.3f, %.3f)" % (transformed_pose.pose.position.x, 
                                                            transformed_pose.pose.position.y, 
                                                            transformed_pose.pose.position.z))
            return transformed_pose
        except Exception as e:
            rospy.logerr("TF transform error: %s", str(e))
            return None

    def open_gripper(self):
        """Open the robot gripper"""
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 0.05  # Open position
        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper opened.")

    def close_gripper(self):
        """Close the robot gripper"""
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 0.01  # Closed position
        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper closed.")

    def lift_torso(self, height=0.4):
        """Lift the robot's torso to the specified height using action client"""
        # Ensure height is within safe limits
        height = max(0.0, min(height, 0.4))
        
        if not self.torso_available:
            rospy.logwarn("Torso controller not available!")
            return False
        
        try:
            # Create trajectory
            trajectory = JointTrajectory()
            trajectory.joint_names = ['torso_lift_joint']
            
            point = JointTrajectoryPoint()
            point.positions = [height]  # Target height
            point.time_from_start = rospy.Duration(3.0)  # 3 seconds to complete
            trajectory.points.append(point)
            
            # Create and send goal
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = trajectory
            
            rospy.loginfo("Moving torso to height %.2f meters..." % height)
            self.torso_client.send_goal(goal)
            
            # Wait for result
            if self.torso_client.wait_for_result(timeout=rospy.Duration(5.0)):
                result = self.torso_client.get_result()
                if result.error_code == result.SUCCESSFUL:
                    rospy.loginfo("Torso moved successfully to %.2f meters!" % height)
                    return True
                else:
                    rospy.logerr("Torso movement failed with error code: %d" % result.error_code)
                    return False
            else:
                rospy.logwarn("Torso movement timed out")
                return False
                
        except Exception as e:
            rospy.logerr("Failed to move torso: %s" % str(e))
            return False

    def lower_torso(self):
        """Lower the robot's torso to minimum position"""
        return self.lift_torso(0.0)

    def create_pose(self, x, y, z):
        """Create a pose message for the robot arm"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Set orientation (no rotation)
        q = quaternion_from_euler(0.0, 0,0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w =1.0
        return pose

    def move_to_pose(self, pose):
        """Move the robot arm to a specified pose"""
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def get_user_color_choice(self):
        """Prompts the user to select a color and validates the input."""
        colors = {
            "1": "green", "2": "light_purple", "3": "baby_blue",
            "4": "purple", "5": "pink", "6": "brown", "7": "mustard"
        }
        
        while True:
            print("\n--- Select a cube color to pick ---")
            for num, color_name in colors.items():
                # Corrected for Python 2.7 string formatting
                print("%s: %s" % (num, color_name.replace('_', ' ').title()))
            choice = raw_input("Enter the number of your choice: ").strip() # raw_input for Python 2.7

            if choice in colors:
                self.selected_color = colors[choice]
                rospy.loginfo("User selected: %s cube." % self.selected_color.replace('_', ' ').title())
                break
            else:
                print("Invalid choice. Please enter a number from the list.")

    def pick_cube(self):
        """Main cube picking function with improved detection and error handling"""
        rospy.loginfo("Starting cube detection and picking...")
        
        # Wait for cube detection
        detection_timeout = 5.0  # seconds to wait for target detection
        start_time = rospy.Time.now().to_sec()
        
        # Wait for target detection
        while self.target_pixel is None:
            if rospy.Time.now().to_sec() - start_time > detection_timeout:
                rospy.logwarn("No %s cube detected within timeout period." % self.selected_color)
                return False
            rospy.sleep(0.5)
            
        rospy.loginfo("Cube detected! Color: %s" % self.detected_color)
        rospy.sleep(1)
        
        # Convert pixel to 3D point
        point = self.pixel_to_3d_point(self.target_pixel)
        if point is None or np.isnan(point[0]):
            rospy.logwarn("Invalid 3D point from point cloud.")
            return False

        # Transform to robot base frame
        transformed = self.transform_point(point)
        if transformed is None:
            rospy.logwarn("Failed to transform point to base_link frame.")
            return False

        x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z

        # Validate if the point is within reasonable bounds
        if x < 0.3 or x > 1.2:
            rospy.logwarn("Cube position out of reasonable range (x=%.2f). Skipping..." % x)
            return False
        
        # Begin grasp sequence
        rospy.loginfo("Starting grasp sequence...")
        
        rospy.sleep(1)

            
        rospy.loginfo("Approached cube from above.")
        rospy.sleep(1)

        # Then move to the actual grasp position
        grasp_pose = self.create_pose(x - 0.15, y, 1.35)
        
        if not self.move_to_pose(grasp_pose):
            rospy.logwarn("Failed to move to grasping position.")
            return False
            
        rospy.loginfo("At grasp position.")
        rospy.sleep(1)
        
        # Close the gripper to grab the cube
        self.open_gripper()
    
        rospy.sleep(1)

        
        return True

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Cube picker started. Looking for cubes...")
        
        # Get user's desired color before starting detection
        self.get_user_color_choice()

        # Set a reasonable frequency for the main loop
        rate = rospy.Rate(10)  # 10 Hz
        success = self.pick_cube()

if __name__ == '__main__':
    try:
        picker = CubePicker()
        picker.run()
    except rospy.ROSInterruptException:
        pass
