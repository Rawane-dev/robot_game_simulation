#!/usr/bin/env python
# -- coding: utf-8 --

import sys
import rospy
import actionlib
import os
import yaml
import moveit_commander
import numpy as np
import cv2
import random
import time
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_ros
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from save_pose import PoseSnapshot
from spawn import spawn_cube


# connect4 game constants
ROWS = 6
COLUMNS = 7
PLAYER_PIECE = 1
AI_PIECE = 2
EMPTY = 0
WINDOW_LENGTH = 4


class Connect4Robot:
    def __init__(self):
        rospy.init_node('connect4_robot', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # initialize board state
        self.board = np.zeros((ROWS, COLUMNS), dtype=int)
        self.game_over = False
        self.turn = AI_PIECE
        self.rounds = 1
        self.round = 0
        self.selected_color = None
        # Initialize ROS components
        self.bridge = CvBridge()
        self.latest_image = None
        self.mode = "idle"
        self.board = np.zeros((6, 7), dtype=int)
        #self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback_above)
        self.pc_sub = rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm_group = self.arm  
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso controller...")
        self.torso_available = self.torso_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Torso controller connected." if self.torso_available else "Torso controller unavailable.")

        self.latest_cloud = None
        self.target_pixel = None
        self.detected_color = None
        self.snap = PoseSnapshot()
	    
    #############################################
    # Robot Motion and Manipulation Functions
    #############################################

    def move_head(self, angle=0.2):
        client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head controller...")
        client.wait_for_server()
        rospy.loginfo("Connected to head controller.")

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, angle]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Head positioned.")

    def move_arm_topdown(self):
        joint_goal = [0.0, 1.1, 0.0, -2.0, 0.0, 0.6, 0.0]
        success = self.arm.go(joint_goal, wait=True)
        self.arm.stop()
        rospy.loginfo("Arm in top-down position." if success else "Arm failed to reach top-down pose.")

    def reach_to_board_safely(self,y):
         pose_target = PoseStamped()
         pose_target.header.frame_id = "base_link"
         pose_target.pose.position.x = 0.65     
         pose_target.pose.position.y = y
         pose_target.pose.position.z = 1.37      
         pose_target.pose.orientation.w = 1.0    

         rospy.loginfo("Moving arm to safe pose in front of the board...")
         self.arm.set_start_state_to_current_state()
         self.arm.set_pose_target(pose_target)

         success = self.arm.go(wait=True)
         self.arm.stop()
         self.arm.clear_pose_targets()

         if success:
            rospy.loginfo("Reached front of the board safely.")
            rospy.sleep(1)
         else:
            rospy.logwarn("Failed to reach pose safely.")
 
    def open_gripper(self):
        """Opens the Fetch gripper."""
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 0.05  
        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper opened.")

    def close_gripper(self):
        joints = self.gripper.get_current_joint_values()
        joints[0] = 0.01
        self.gripper.go(joints, wait=True)
        self.gripper.stop()
        rospy.loginfo("Gripper closed.")

    
    
    def lift_torso(self, height):
        height = max(0.0, min(height, 0.39))
        if not self.torso_available:
            return False
        try:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['torso_lift_joint']
            point = JointTrajectoryPoint()
            point.positions = [height]
            point.time_from_start = rospy.Duration(3.0)
            goal.trajectory.points.append(point)

            self.torso_client.send_goal(goal)
            self.torso_client.wait_for_result(rospy.Duration(5.0))
            result = self.torso_client.get_result()
            return result.error_code == result.SUCCESSFUL
        except Exception as e:
            rospy.logerr("Torso error: %s", str(e))
            return False

    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(0.0, 0, 0)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q #w=1.0
        return pose

    def move_to_pose(self, pose):
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    #############################################
    # Vision and Perception Functions
    #############################################
    
    def pc_callback(self, msg):
        self.latest_cloud = msg

    def detect_color(self,bgr):
       b, g, r = bgr
       if r > 120 and g < 100 and b < 100:
         return 1  # Red
       elif r > 150 and g > 150 and b < 120:
         return 2  # Yellow
       else:
         return 0  # Empty
       

    def detect_board_state_from_camera(self,image, coords_file_path):
      """
      Reads a camera image and coordinate list, then returns a 6x7 matrix
      representing the current game board state using color detection.
    
      Parameters:
        image: OpenCV image (numpy array) from the camera
        coords_file_path: path to YAML file containing grid coordinates

      Returns:
        A 6x7 list of lists with values:
            0 = empty
            1 = red
            2 = yellow
       """
      try:
        # Load coordinates from YAML
        with open(coords_file_path, 'r') as f:
            data = yaml.safe_load(f)

        coords = data.get("grid_coords", []) if isinstance(data, dict) else data
        if not coords or len(coords) != 42:
            rospy.logwarn("⚠️ Expected 42 grid coordinates in YAML.")
            return None

        # Initialize empty 6x7 board
        grid = [[0 for _ in range(7)] for _ in range(6)]
        
        # Sample each coordinate
        for idx, coord in enumerate(coords):
            if not isinstance(coord, (list, tuple)) or len(coord) != 2:
                rospy.logwarn("Invalid coordinate format at index %d", idx)
                continue

            x, y = coord
            if y >= image.shape[0] or x >= image.shape[1]:
                rospy.logwarn("Coordinate (%d, %d) out of image bounds.", x, y)
                continue

            bgr = image[y, x]
            color_code = self.detect_color(bgr)

            row = idx // 7
            col = idx % 7
            grid[row][col] = color_code

            rospy.loginfo("Cell (%d, %d) → BGR: %s → Code: %d", row, col, str(bgr), color_code)
            #image_visu = image.copy()
            #cv2.imshow("Grille Puissance4", image_visu)
            #cv2.waitKey(1)
            
        return grid
        

      except Exception as e:
         rospy.logerr("❌ Error in detect_board_state_from_camera: %s", str(e))
         return None

    def image_callback_above(self, msg):
      """Unified callback for both cube detection and board detection."""
      try:
        # convert incoming ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = frame  
      except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))
        return

      
      

      # --- Cube Detection Mode ---
      if self.mode != "cube_detection" or self.selected_color is None:
        return  # Skip if cube detection not requested

      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

      color_ranges = {
        "green": [(np.array([45, 100, 100]), np.array([75, 255, 255]))],
        "baby_blue": [(np.array([85, 50, 100]), np.array([105, 255, 255]))],
        "light_purple": [(np.array([135, 60, 130]), np.array([155, 160, 255]))],
        "purple": [(np.array([125, 100, 100]), np.array([155, 255, 255]))],
        "pink": [(np.array([160, 100, 100]), np.array([175, 255, 255]))],
        "brown": [(np.array([10, 100, 20]), np.array([20, 255, 200]))],
        "mustard": [(np.array([20, 150, 150]), np.array([30, 255, 200]))],
        "yellow": [(np.array([25, 100, 100]), np.array([35, 255, 255]))],
        "red": [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([160, 100, 100]), np.array([180, 255, 255]))
        ]
      }

      for lower, upper in color_ranges.get(self.selected_color, []):
        mask = cv2.inRange(hsv, lower, upper)
        try:
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        except ValueError:
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                x, y, w, h = cv2.boundingRect(largest)
                cx, cy = x + w // 2, y + h // 2
                self.target_pixel = (cx, cy)
                self.detected_color = self.selected_color
                rospy.loginfo("🎯 Detected a %s cube at (%d, %d)", self.selected_color, cx, cy)
                return  # exit after first match
    


    def pixel_to_3d_point(self, pixel):
        if self.latest_cloud is None:
            return None
        u, v = pixel
        gen = pc2.read_points(self.latest_cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=[(u, v)])
        return next(gen, None)

    def transform_point(self, point):
        pose = PoseStamped()
        pose.header.frame_id = self.latest_cloud.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
        pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform("base_link", pose.header.frame_id,
                                                        rospy.Time(0), rospy.Duration(1.0))
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except Exception as e:
            rospy.logerr("Transform error: %s", str(e))
            return None

    def pick_cube(self):
        """Main cube picking function with improved detection and error handling"""
        rospy.loginfo("Starting cube detection and picking...")
        
        # wait for cube detection
        detection_timeout = 5.0  
        start_time = rospy.Time.now().to_sec()
        
        # Wait for target detection
        while self.target_pixel is None:
            if rospy.Time.now().to_sec() - start_time > detection_timeout:
                rospy.logwarn("No cube detected within timeout period.")
                return False
            rospy.sleep(0.5)
            
        rospy.loginfo("Cube detected! Color: %s", self.detected_color)
        rospy.sleep(1)
        
        # convert pixel to 3D point
        point = self.pixel_to_3d_point(self.target_pixel)
        if point is None or np.isnan(point[0]):
            rospy.logwarn("Invalid 3D point from point cloud.")
            return False

        # transform to robot base frame
        transformed = self.transform_point(point)
        if transformed is None:
            rospy.logwarn("Failed to transform point to base_link frame.")
            return False

        x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z

        # validate if the point is within reasonable bounds
        if x < 0.3 or x > 1.2:
            rospy.logwarn("Cube position out of reasonable range (x=%.2f). Skipping...", x)
            return False
        
        # begin grasp sequence
        rospy.loginfo("Starting grasp sequence...")
        self.open_gripper()
        rospy.sleep(1)

        # we first move to a position above the cube
        above_pose = self.create_pose(0.5, y, z + 0.07)
        if not self.move_to_pose(above_pose):
            rospy.logwarn("Failed to move to position cube.")
            return False
            
        rospy.loginfo("Approached cube from above.")
        rospy.sleep(1)

        # then move to the actual grasp position
        grasp_pose = self.create_pose(x - 0.12, y, 0.78)
        if not self.move_to_pose(grasp_pose):
            rospy.logwarn("Failed to move to grasping position.")
            return False
            
        rospy.loginfo("At grasp position.")
        rospy.sleep(1)
        
        # close the gripper to grab the cube
        self.close_gripper()
        rospy.sleep(1)

        # lift the torso to maximum height (0.4m) after grasping
        rospy.loginfo("Lifting torso to maximum height after grasping cube...")
        if self.lift_torso(0.4):
           rospy.loginfo("Torso lifted successfully to 0.4m")
           rospy.sleep(2)  # pause for a moment
        else:
           rospy.logwarn("Failed to lift torso")
            
        # get to the board
        self.reach_to_board_safely(0.0)
        
        # reset target pixel after successful grasp
        self.target_pixel = None
        return True
    def get_user_color_choice(self,col):
        """Prompts the user to select a color and validates the input."""
        colors = {
            0: "green", 1: "light_purple", 2: "baby_blue",
            3: "purple", 4: "pink", 5: "brown", 6: "mustard", 8:"yellow",9:"red"
        }
 
        choice = col

        if choice in colors:
            self.selected_color = colors[choice]
            rospy.loginfo("User selected: %s cube." % self.selected_color.replace('_', ' ').title())
            
        else:
            print("Invalid choice. Please enter a number from the list.")

    def reach_cube(self):
        """Main cube picking function with improved detection and error handling"""
        rospy.loginfo("Starting cube detection and picking...")
        
        # wait for cube detection
        detection_timeout = 5.0  
        start_time = rospy.Time.now().to_sec()
        
       
        while self.target_pixel is None:
            if rospy.Time.now().to_sec() - start_time > detection_timeout:
                rospy.logwarn("No %s cube detected within timeout period." % self.selected_color)
                return False
            rospy.sleep(0.5)
            
        rospy.loginfo("Cube detected! Color: %s" % self.detected_color)
        rospy.sleep(1)
        
        # convert pixel to 3D point
        point = self.pixel_to_3d_point(self.target_pixel)
        if point is None or np.isnan(point[0]):
            rospy.logwarn("Invalid 3D point from point cloud.")
            return False

        # transform to robot base frame
        transformed = self.transform_point(point)
        if transformed is None:
            rospy.logwarn("Failed to transform point to base_link frame.")
            return False

        x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z

        # validate if the point is within reasonable bounds
        if x < 0.3 or x > 1.2:
            rospy.logwarn("Cube position out of reasonable range (x=%.2f). Skipping..." % x)
            return False
              
        rospy.loginfo("Approached the column.")

        # then move to the actual grasp position
        grasp_pose = self.create_pose(x - 0.14, y, 1.35)
        
        if not self.move_to_pose(grasp_pose):
            rospy.logwarn("Failed to move to the column position.")
            return False
            
        rospy.loginfo("At column position.")
        rospy.sleep(3)
        
        # close the gripper to grab the cube
        self.open_gripper()
    
        rospy.sleep(3)

        
        return True

    #############################################
    # Connect4 Game Logic Functions
    #############################################

    def print_board(self):
        """Print the game board (flipped for correct display)."""
        print(self.board[::-1])

    def is_valid_move(self, col):
        """Check if a move is valid (column is not full)."""
        return self.board[ROWS-1, col] == 0

    def get_next_open_row(self, col):
        """Find the lowest empty row in the selected column."""
        for row in range(ROWS):
            if self.board[row, col] == 0:
                return row
        return None

    def drop_piece(self, row, col, piece):
        """Place the player's piece (1 or 2) in the board."""
        self.board[row, col] = piece

    def check_win(self, piece):
        """Check if the player wins the game."""
        # Horizontal
        for r in range(ROWS):
            for c in range(COLUMNS - 3):
                if all(self.board[r, c + i] == piece for i in range(4)):
                    return True

        # Vertical
        for r in range(ROWS - 3):
            for c in range(COLUMNS):
                if all(self.board[r + i, c] == piece for i in range(4)):
                    return True

        # Positive diagonal
        for r in range(ROWS - 3):
            for c in range(COLUMNS - 3):
                if all(self.board[r + i, c + i] == piece for i in range(4)):
                    return True

        # Negative diagonal
        for r in range(3, ROWS):
            for c in range(COLUMNS - 3):
                if all(self.board[r - i, c + i] == piece for i in range(4)):
                    return True

        return False

    def get_valid_locations(self):
        return [col for col in range(COLUMNS) if self.is_valid_move(col)]

    def is_terminal_node(self):
        return self.check_win(PLAYER_PIECE) or self.check_win(AI_PIECE) or len(self.get_valid_locations()) == 0

    def evaluate_window(self, window, piece):
        score = 0
        opp_piece = PLAYER_PIECE if piece == AI_PIECE else AI_PIECE

        if window.count(piece) == 4:
            score += 100
        elif window.count(piece) == 3 and window.count(EMPTY) == 1:
            score += 5
        elif window.count(piece) == 2 and window.count(EMPTY) == 2:
            score += 2

        if window.count(opp_piece) == 3 and window.count(EMPTY) == 1:
            score -= 4

        return score

    def score_position(self, piece):
        score = 0

        # center column preference
        center_array = [int(i) for i in list(self.board[:, COLUMNS // 2])]
        center_count = center_array.count(piece)
        score += center_count * 3

        # horizontal
        for r in range(ROWS):
            row_array = [int(i) for i in list(self.board[r, :])]
            for c in range(COLUMNS - 3):
                window = row_array[c:c + WINDOW_LENGTH]
                score += self.evaluate_window(window, piece)

        # vertical
        for c in range(COLUMNS):
            col_array = [int(i) for i in list(self.board[:, c])]
            for r in range(ROWS - 3):
                window = col_array[r:r + WINDOW_LENGTH]
                score += self.evaluate_window(window, piece)

        # positive diagonal
        for r in range(ROWS - 3):
            for c in range(COLUMNS - 3):
                window = [self.board[r + i][c + i] for i in range(WINDOW_LENGTH)]
                score += self.evaluate_window(window, piece)

        # negative diagonal
        for r in range(3, ROWS):
            for c in range(COLUMNS - 3):
                window = [self.board[r - i][c + i] for i in range(WINDOW_LENGTH)]
                score += self.evaluate_window(window, piece)

        return score

    def minimax(self, depth, alpha, beta, maximizingPlayer):
        valid_locations = self.get_valid_locations()
        terminal = self.is_terminal_node()

        if depth == 0 or terminal:
            if terminal:
                if self.check_win(AI_PIECE):
                    return (None, float('inf'))
                elif self.check_win(PLAYER_PIECE):
                    return (None, -float('inf'))
                else:
                    return (None, 0)
            else:
                return (None, self.score_position(AI_PIECE))

        if maximizingPlayer:
            value = -float('inf')
            best_col = random.choice(valid_locations)
            for col in valid_locations:
                row = self.get_next_open_row(col)
                
                # create a copy of the board for simulation
                b_copy = np.copy(self.board)
                
                # temporarily make the move on the copy
                b_copy[row, col] = AI_PIECE
                
                # create a temporary board state for recursion
                temp_board = self.board
                self.board = b_copy
                
                # run minimax on this hypothetical board state
                new_score = self.minimax(depth-1 , alpha, beta, False)[1]
                
                # restore the original board
                self.board = temp_board
                
                if new_score > value:
                    value = new_score
                    best_col = col
                alpha = max(alpha, value)
                if alpha >= beta:
                    break
            return best_col, value
        else:
            value = float('inf')
            best_col = random.choice(valid_locations)
            for col in valid_locations:
                row = self.get_next_open_row(col)
                
                # create a copy of the board for simulation
                b_copy = np.copy(self.board)
                
                # temporarily make the move on the copy
                b_copy[row, col] = PLAYER_PIECE
                
                # create a temporary board state for recursion
                temp_board = self.board
                self.board = b_copy
                
                # run minimax on this hypothetical board state
                new_score = self.minimax(depth - 1, alpha, beta, True)[1]
                
                # restore the original board
                self.board = temp_board
                
                if new_score < value:
                    value = new_score
                    best_col = col
                beta = min(beta, value)
                if alpha >= beta:
                    break
            return best_col, value

    def robot_play_turn(self, col):
     """Physical robot places a piece in the specified column"""
     rospy.sleep(1)
     rospy.loginfo("Robot moving to place piece in column %d", col)
     self.snap.restore_pose("pregrasp_pose")
     self.move_head(0.2)
     spawn_cube(9)
     self.get_user_color_choice(8)
     success = self.pick_cube()
     if not success:
        rospy.logerr("Failed to pick up a game piece!")
        return False

     rospy.sleep(1)

     if col == 0:
        self.reach_to_board_safely(-0.5)
        rospy.sleep(1)
     elif col == 6:
        self.reach_to_board_safely(0.5)
     elif col == 4:
        self.reach_to_board_safely(0.0)
     self.move_head(0.0)
     rospy.sleep(1)
     self.get_user_color_choice(col)
     col_success = self.reach_cube()
     rospy.sleep(1)
     if not col_success:
        rospy.logerr("Failed to move to column %d", col)
        return False

     rospy.sleep(1)
     rospy.loginfo("Successfully placed piece in column %d", col)
     self.snap.restore_pose("hito_pose")
     rospy.sleep(1)
     self.move_head(0.0)
     
     rospy.sleep(3)

     return True
            
       

    def run_game(self):
        """Main Connect 4 game loop with robot interaction"""
        rospy.loginfo("Welcome to Connect 4 with Robot!")
       
        
        # track game state
        self.game_over = False
        self.round = 0
        
        while not self.game_over:
            if self.round == 2:
                self.rounds += 1
                print("Round:", self.rounds)
                self.round = 0

            if self.turn == PLAYER_PIECE:
                self.round += 1
                print("\nYour Turn (You are Player 1)")
                try:
                    col = int(raw_input("Choose a column (0-6): "))
                except:
                    print("Invalid input. Enter an integer between 0 and 6.")
                    continue

                if col < 0 or col >= COLUMNS:
                    print("Invalid column. Choose between 0 and 6.")
                    continue

                if self.is_valid_move(col):
                    row = self.get_next_open_row(col)
                    self.drop_piece(row, col, PLAYER_PIECE)
                    print("You placed a piece in column {}".format(col))
                    spawn_cube(col)
                    time.sleep(0.5)

                    if self.check_win(PLAYER_PIECE):
                        #self.print_board()
                        print("\nYOU WIN!")
                        self.game_over = True
                else:
                    print("Column is full. Choose another.")
                    continue

            else:  # AI's turn
                
                self.round += 1
                print("\nRobot (Player 2) is analyzing the board...\n")
                self.move_head(0.0)
                rospy.sleep(1)
                self.snap.restore_pose("detection_pose")
                self.mode = "board_detection"
                # Detect board from camera image
                if self.latest_image is not None:
                 coords_path = os.path.join(os.path.dirname(__file__), "grid_coords.yaml")
                 detected_board= self.detect_board_state_from_camera(self.latest_image,coords_path)
                 print("Detected board:\n", detected_board)
                 
                 
                 self.board = np.array(detected_board)
                 self.print_board()
                 
                else:
                 print("⚠ No image from camera. Skipping detection.")
                 continue
                # use minimax algorithm to determine the best move
                col, minimax_score = self.minimax(4, -float('inf'), float('inf'), True)
                print("Robot chose column {}".format(col))

                if self.is_valid_move(col):
                    row = self.get_next_open_row(col)
                    #update the internal board state
                    self.drop_piece(row, col, AI_PIECE)
                    self.mode = "cube_detection"
                    #execute the robot's physical move
                    self.robot_play_turn(col)
                    rospy.sleep(1)
                    #check for win after the robot's move
                    if self.check_win(AI_PIECE):
                        self.print_board()
                        print("\nROBOT WINS!")
                        self.game_over = True

            
            time.sleep(1)

            if not self.game_over:
                self.turn += 1
                self.turn = self.turn % 2

        print("Game over!")


if __name__ == '__main__':
    try: 
        robot = Connect4Robot()
        robot.run_game()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
