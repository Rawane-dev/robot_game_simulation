#!/usr/bin/env python

import rospy
import moveit_commander

def reset_arm():
    rospy.init_node("reset_arm_position")
    moveit_commander.roscpp_initialize([])

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_planner_id("RRTConnectkConfigDefault")
    arm.set_num_planning_attempts(10)
    arm.set_planning_time(5)
    arm.allow_replanning(True)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)

    # Define a tucked, default position (no torso movement)
    joint_goal = [
        1.32,      # shoulder_pan_joint
        1.4,      # shoulder_lift_joint
        -0.20,      # upperarm_roll_joint
        1.72,    # elbow_flex_joint
        0.0,      # forearm_roll_joint
        1.66,      # wrist_flex_joint
        0.0       # wrist_roll_joint
    ]

    rospy.loginfo("Resetting arm to initial folded position...")
    arm.go(joint_goal, wait=True)
    arm.stop()
    rospy.loginfo("Arm reset complete.")

if __name__ == "__main__":
    try:
        reset_arm()
    except rospy.ROSInterruptException:
        pass
