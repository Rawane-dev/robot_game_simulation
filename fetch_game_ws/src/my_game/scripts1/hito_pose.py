#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def open_gripper(gripper_group):
    """Opens the Fetch gripper."""
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.05  # Adjust width if needed
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

def reach_to_board_safely():
    rospy.init_node("fetch_safe_reach_pose")
    moveit_commander.roscpp_initialize([])

    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # Optional: increase success rate
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    arm_group.set_num_planning_attempts(10)
    arm_group.set_planning_time(5.0)
    arm_group.allow_replanning(True)

    # Set target pose: close to the board (0.5m ahead), gripper centered and raised
    pose_target = PoseStamped()
    pose_target.header.frame_id = "base_link"
    pose_target.pose.position.x = 0.65      # Keep a safe distance from board
    pose_target.pose.position.y = 0.5
    pose_target.pose.position.z = 1.37      # Gripper height
    pose_target.pose.orientation.w = 1.0    # Simple forward-facing orientation

    rospy.loginfo("Moving arm to safe pose in front of the board...")

    arm_group.set_start_state_to_current_state()
    arm_group.set_pose_target(pose_target)

    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    if success:
        rospy.loginfo("Reached front of the board safely.")
        rospy.sleep(1)
       
    else:
        rospy.logwarn("Failed to reach pose safely.")

if __name__ == '__main__':
    try:
        reach_to_board_safely()
    except rospy.ROSInterruptException:
        pass
