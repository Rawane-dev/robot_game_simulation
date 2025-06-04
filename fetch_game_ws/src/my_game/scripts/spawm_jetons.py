#!/usr/bin/env python
# -*- coding: utf-8 

import rospy
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_jeton(nom, path, x, y, z):
    with open(path, 'r') as file:
        model_xml = file.read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))
        spawn_model(model_name=nom, model_xml=model_xml, robot_namespace='', initial_pose=pose, reference_frame='world')
        rospy.loginfo("Jeton {} spawné à {}, {}, {}".format(nom, x, y, z))
    except Exception as e:
        rospy.logerr("Erreur lors du spawn de {} : {}".format(nom, e))

if __name__ == '__main__':
    rospy.init_node('spawn_jetons_node')

    model_dir = os.path.join(os.getenv('HOME'), 'catkin_ws', 'src', 'puissance4_sim', 'models')

    spawn_jeton('jeton_rouge2', os.path.join(model_dir, 'jeton_rouge', 'model.sdf'), 0.5, 0.7, 0.805)
    spawn_jeton('jeton_jaune2', os.path.join(model_dir, 'jeton_jaune', 'model.sdf'), 0.9, 0.9, 0.805)


