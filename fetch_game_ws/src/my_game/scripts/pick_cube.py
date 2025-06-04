#!/usr/bin/env python
# -*- coding: utf-8

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import sys

def move_to_cube(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fetch_pick_cube', anonymous=True)

    arm = moveit_commander.MoveGroupCommander("arm_with_torso")
    gripper = moveit_commander.MoveGroupCommander("gripper")

    # Optionnel : ouvrir le gripper avant de s'approcher
    gripper.set_named_target("open")
    gripper.go(wait=True)

    # Pose du cube (approche au-dessus du cube)
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z + 0.1  # approche 10 cm au-dessus

    # Orientation neutre (main vers le bas)
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    arm.set_pose_target(target_pose)
    arm.go(wait=True)

    rospy.sleep(1.0)

    # Descente (vers le cube)
    target_pose.pose.position.z = z
    arm.set_pose_target(target_pose)
    arm.go(wait=True)

    rospy.sleep(1.0)

    # Fermer le gripper
    gripper.set_named_target("close")
    gripper.go(wait=True)

    rospy.loginfo("Cube saisi !")

if __name__ == '__main__':
    try:
        # Exemple : coordonnées détectées du cube
        x, y, z = 0.05, 0.0, 0.60
        move_to_cube(x, y, z)
    except rospy.ROSInterruptException:
        pass


