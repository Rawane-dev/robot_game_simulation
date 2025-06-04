#!/usr/bin/env python
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

def lift_torso_direct():
    rospy.init_node('direct_torso_control')
    
    # Create action client
    client = actionlib.SimpleActionClient(
        '/torso_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)
    client.wait_for_server()
    
    # Create trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = ['torso_lift_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.0]  # Target height
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)
    
    # Create and send goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    
    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()

if __name__ == '__main__':
    try:
        result = lift_torso_direct()
        if result.error_code == result.SUCCESSFUL:
            rospy.loginfo("Torso moved successfully!")
        else:
            rospy.logerr("Movement failed with error code: %d", result.error_code)
    except rospy.ROSInterruptException:
        pass
