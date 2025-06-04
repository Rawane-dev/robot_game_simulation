#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm(joint_positions, duration=3.0):
    """Move Fetch's arm to a specific position."""
    rospy.init_node('fetch_arm_mover', anonymous=True)

    # Create action client for the arm controller
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    # Create trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
    ]

    # Create a single trajectory point
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(duration)  # Move in 'duration' seconds

    # Add the point to the trajectory
    goal.trajectory.points.append(point)

    # Send goal to the action server
    rospy.loginfo("Moving Fetch's arm to: %s", joint_positions)
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == "__main__":
    try:
        # Example: Move arm up
        move_arm([0.0, 0.5, 0.0, -1.5, 0.0, 1.0, 0.0], 3)  # Moves arm to a lifted position
        rospy.sleep(2)
        # Example: Move arm down
        move_arm([0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 3)  # Moves arm down
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm movement interrupted.")

