#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_forward_and_look_down():
    rospy.init_node('fetch_move_and_look')

    # Publisher pour la base
    cmd_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)

    # Publisher pour la tête
    head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1.0)  # Attendre les connexions

    # Commande pour incliner la tête
    traj = JointTrajectory()
    traj.joint_names = ['head_pan_joint', 'head_tilt_joint']
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.9]  # pan=0 (face), tilt=-0.9 rad (vers le bas)
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)
    head_pub.publish(traj)

    # Commande pour faire avancer le robot
    move_cmd = Twist()
    move_cmd.linear.x = 0.4
    move_cmd.angular.z = 0.0

    duration = 6  # secondes d'avance
    rate = rospy.Rate(10)
    ticks = int(duration * 10)

    rospy.loginfo("Début : avancer + incliner la tête...")
    for _ in range(ticks):
        cmd_pub.publish(move_cmd)
        rate.sleep()

    # Stop
    move_cmd.linear.x = 0.0
    cmd_pub.publish(move_cmd)
    rospy.loginfo("Arrêté.")

if __name__ == '__main__':
    try:
        move_forward_and_look_down()
    except rospy.ROSInterruptException:
        pass

