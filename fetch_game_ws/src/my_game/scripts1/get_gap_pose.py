#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def open_gripper(gripper_group):
    """Opens the Fetch gripper."""
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.05  # Gripper finger joint
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

def move_to_connect4_column(column_index):
    # Predefined (x, y, z) positions for each Connect 4 column
    column_positions = [
        (0.78, -0.3, 1.35),  # Column 0
        (0.78, -0.2, 1.35),  # Column 1
         (0.78, -0.11, 1.35),  # Column 2
        (0.78, -0.03, 1.35), # Column 3
        (0.78, 0.05, 1.35), # Column 4
        (0.78, 0.14, 1.35),  # Column 5
        (0.78, 0.24, 1.35)   # Column 6
    ]

    if not (0 <= column_index < len(column_positions)):
        rospy.logerr("Invalid column index: {}. Must be between 0 and 6.".format(column_index))
        return False

    x, y, z = column_positions[column_index]
    rospy.loginfo("Moving to Connect 4 column {}".format(column_index))

    # Create pose target
    pose_target = PoseStamped()
    pose_target.header.frame_id = "base_link"
    pose_target.pose.position.x = x
    pose_target.pose.position.y = y
    pose_target.pose.position.z = z
    pose_target.pose.orientation.w = 1.0  # Neutral orientation

    # Move to target
    arm_group.set_start_state_to_current_state()
    arm_group.set_pose_target(pose_target)

    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    if success:
        rospy.loginfo("Reached column {}".format(column_index))
        rospy.sleep(1)
        open_gripper(gripper_group)
        return True
    else:
        rospy.logwarn("Failed to move to column {}".format(column_index))
        return False

if __name__ == '__main__':
    try:
        rospy.init_node("fetch_move_to_connect4_column")
        moveit_commander.roscpp_initialize([])

        arm_group = moveit_commander.MoveGroupCommander("arm")
        gripper_group = moveit_commander.MoveGroupCommander("gripper")

        column_index = int(input("Enter Connect 4 column number (0-6): "))
        move_to_connect4_column(column_index)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
