#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def open_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.05  # Open width
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

def close_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.0  # Fully closed
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    rospy.init_node('arm_control_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Control the gripper
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    rospy.sleep(1)  # Give time to initialize

    close_gripper(gripper_group)
