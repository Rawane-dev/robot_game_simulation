#!/usr/bin/env python
# -*- coding: utf-8

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_fetch_arm():
    rospy.init_node('move_fetch_arm_simple')

    # Liste des joints du bras du Fetch
    joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upperarm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint'
    ]

    # Création du client d'action
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Attente de l'action server...")
    client.wait_for_server()
    rospy.loginfo("Action server trouvé.")

    # Création de la cible (valeurs d'exemple)
    target_positions = [0.0, -1.0, 0.0, 1.5, 0.0, 1.0, 0.0]

    # Construction de l'objectif
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = target_positions
    point.time_from_start = rospy.Duration(3.0)  # 3 secondes pour atteindre la position

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

    rospy.loginfo("Envoi de la commande au bras...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Commande terminée.")

if __name__ == '__main__':
    try:
        move_fetch_arm()
    except rospy.ROSInterruptException:
        pass


