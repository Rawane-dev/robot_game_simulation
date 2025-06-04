#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PointStamped

class GoToCube:
    def __init__(self):
        rospy.init_node('go_to_cube')
        self.cmd_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        rospy.Subscriber('/red_cube_position', PointStamped, self.cube_callback)

        self.speed = 0.2  # m/s
        self.offset = 0.5  # stop à 0.5 m du cube

        rospy.loginfo("go_to_cube prêt. En attente de /red_cube_position...")
        rospy.spin()

    def cube_callback(self, msg):
        cube_x = msg.point.x
        cube_y = msg.point.y
        cube_z = msg.point.z

        target_distance = cube_x - self.offset
        if target_distance <= 0:
            rospy.logwarn("Le cube est trop proche ou déjà dépassé (distance = %.2f m)", target_distance)
            return

        rospy.loginfo("Cube détecté à %.2f m. Avancer de %.2f m...", cube_x, target_distance)

        move_cmd = Twist()
        move_cmd.linear.x = self.speed

        duration = target_distance / self.speed  # temps = distance / vitesse
        rate = rospy.Rate(10)
        ticks = int(duration * 10)

        for _ in range(ticks):
            self.cmd_pub.publish(move_cmd)
            rate.sleep()

        # arrêt
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)
        rospy.loginfo("Arrivé à %.2f m du cube. Arrêt.", self.offset)

        # Se désabonner après un seul déplacement
        rospy.signal_shutdown("Déplacement terminé.")

if __name__ == '__main__':
    try:
        GoToCube()
    except rospy.ROSInterruptException:
        pass


	

