#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def move_fetch_head(angle=-0.5):  # Negative looks down
    rospy.init_node('move_fetch_head_down')

    client = actionlib.SimpleActionClient(
        '/head_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    rospy.loginfo("Waiting for head trajectory controller...")
    client.wait_for_server()
    rospy.loginfo("Connected to head controller.")

    # Build the trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']

    point = JointTrajectoryPoint()
    point.positions = [0.0, angle]  # head_pan = 0.0, head_tilt = angle
    point.time_from_start = rospy.Duration(2.0)  # duration to reach this point

    goal.trajectory.points = [point]
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Head movement done.")

if __name__ == '__main__':
    try:
        move_fetch_head(0.0)
    except rospy.ROSInterruptException:
        pass
