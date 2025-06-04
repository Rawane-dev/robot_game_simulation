#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
import moveit_commander
import sys

class GrabRedCube:
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('grab_red_cube_arm')

        # Initialisation de MoveIt! pour le bras et la pince du robot
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")

        # Souscrire aux coordonnées du cube rouge
        rospy.Subscriber('/red_cube_position', PointStamped, self.cube_callback)
       
        rospy.loginfo("En attente des coordonnées du cube rouge...")
        rospy.spin()

    def cube_callback(self, msg):
        # Récupérer les coordonnées du cube
        cube_x = msg.point.x
        cube_y = msg.point.y
        cube_z = msg.point.z

        rospy.loginfo("Cube détecté à : x=%s, y=%s, z=%s" % (cube_x, cube_y, cube_z))

        # Positionner le bras au-dessus du cube
        self.move_arm_to_cube(cube_x, cube_y, cube_z)

        # Fermer la pince pour saisir le cube
        self.close_gripper()

        rospy.loginfo("Cube saisi avec succès.")

    def move_arm_to_cube(self, cube_x, cube_y, cube_z):
        # Positionner le bras pour être légèrement au-dessus du cube
        target_position = [cube_x, cube_y, cube_z + 0.1]  # Ajouter un décalage sur l'axe Z pour être au-dessus du cube
        self.arm_group.set_position_target(target_position)
        self.arm_group.go()
        rospy.sleep(2)  # Attendre que le bras atteigne la position

    def close_gripper(self):
        # Fermer la pince pour saisir le cube
        self.gripper_group.set_named_target("close")
        self.gripper_group.go()
        rospy.sleep(2)  # Attendre que la pince soit fermée

if __name__ == '__main__':
    try:
        GrabRedCube()
    except rospy.ROSInterruptException:
        pass


