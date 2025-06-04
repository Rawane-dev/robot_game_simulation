#!/usr/bin/env python

import sys
import rospy
import moveit_commander

def open_fetch_arm_topdown():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('open_fetch_arm_topdown', anonymous=True)

    arm_group = moveit_commander.MoveGroupCommander("arm")

    arm_group.set_planning_time(10)
    arm_group.set_num_planning_attempts(5)

   
    joint_goal = [
          -1.0,      # shoulder_pan_joint
          1.1,     # shoulder_lift_joint
          0.0,      # upperarm_roll_joint
         -2.0,      # elbow_flex_joint
         0.0,      # forearm_roll_joint
          0.6,     # wrist_flex_joint
          0.0       # wrist_roll_joint
             ]

    success = arm_group.go(joint_goal, wait=True)
    arm_group.stop()

    if success:
        rospy.loginfo("Fetch's arm is now in top-down position.")
    else:
        rospy.logwarn("Failed to move Fetch's arm.")

if __name__ == '__main__':
    try:
        open_fetch_arm_topdown()
    except rospy.ROSInterruptException:
        pass
