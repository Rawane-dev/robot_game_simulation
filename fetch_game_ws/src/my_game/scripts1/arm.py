#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import moveit_commander

def open_fetch_arm_topdown():
    # DO NOT initialize node or moveit again — it's already done by the main script
    arm_group = moveit_commander.MoveGroupCommander("arm")

    arm_group.set_planning_time(10)
    arm_group.set_num_planning_attempts(5)

    joint_goal = [
        0.0,   # shoulder_pan_joint
        1.1,   # shoulder_lift_joint
        0.0,   # upperarm_roll_joint
       -2.0,   # elbow_flex_joint
        0.0,   # forearm_roll_joint
        0.9,   # wrist_flex_joint
        0.0    # wrist_roll_joint
    ]

    success = arm_group.go(joint_goal, wait=True)
    arm_group.stop()

    if success:
        rospy.loginfo("Fetch's arm is now in top-down position.")
    else:
        rospy.logwarn("Failed to move Fetch's arm.")
