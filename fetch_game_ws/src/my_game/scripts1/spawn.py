#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import os
import time

def spawn_cube(column):
    pose = Pose()

    if column == 0 or column ==15:
        pose.position.x = 1.63872492313
        pose.position.y = -0.398097664118
        pose.position.z = 0.9855045795
        pose.orientation.x = 7.37897321199e-06
        pose.orientation.y = 0.000534024603415
        pose.orientation.z = 0.0108684699804
        pose.orientation.w = 0.999940793809

    elif column == 1 or column ==16:
        pose.position.x = 1.6371935606
        pose.position.y = -0.296382576227
        pose.position.z = 1.30111157894
        pose.orientation.x = 0.999982273489
        pose.orientation.y = 0.0040900454809
        pose.orientation.z = 0.00411789714749
        pose.orientation.w = -0.00132934501473

    elif column == 2 or column == 12:
        pose.position.x = 1.63958585262
        pose.position.y = -0.193260610104
        pose.position.z = 1.79187464714
        pose.orientation.x = -0.999990983121
        pose.orientation.y = -0.00409695639002
        pose.orientation.z = -0.000710192241108
        pose.orientation.w = -0.000862700850371

    elif column == 3 or column == 10 :
        pose.position.x = 1.64115726948
        pose.position.y = -0.0968464687467
        pose.position.z = 1.80187857151
        pose.orientation.x = 0.999990905409
        pose.orientation.y = 0.00409659334422
        pose.orientation.z = -0.000713174132707
        pose.orientation.w = -0.000947842339856

    elif column == 4 or column == 11:
        pose.position.x = 1.6411832571
        pose.position.y = -0.0030825403519
        pose.position.z = 1.30072665215
        pose.orientation.x = -0.999991294242
        pose.orientation.y = -0.00409860628162
        pose.orientation.z = 0.000770587937087
        pose.orientation.w = -0.000138062328435

    elif column == 5 or column ==13:
        pose.position.x = 1.63796031475
        pose.position.y = 0.0917639285326
        pose.position.z = 1.79471027851
        pose.orientation.x = 0.999991303402
        pose.orientation.y = 0.00409887547139
        pose.orientation.z = 0.000763698368946
        pose.orientation.w = 9.542040843e-05

    elif column == 6 or column ==14:
        pose.position.x = 1.63973510265
        pose.position.y = 0.199451312423
        pose.position.z = 1.80216062069
        pose.orientation.x = 0.999991353145
        pose.orientation.y = 0.00409897089153
        pose.orientation.z = -0.000701440146054
        pose.orientation.w = -7.35395948704e-06

    elif column == 9 :
        pose.position.x = 1.4610879031
        pose.position.y = -0.282990753059
        pose.position.z = 0.770050502984
        pose.orientation.x = 2.87581038852e-06
        pose.orientation.y = 0.000517026450595
        pose.orientation.z = -0.0171769705827
        pose.orientation.w = 0.999852331275

    else:
        raise ValueError("Invalid column. Must be between 0 and 6 or 9.")

    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # Choose model path based on column
        if column >= 9 :
            model_path = os.path.join(os.environ['HOME'], '.gazebo', 'models', 'cube_colored', 'cube_yellow', 'model.sdf')
        else:
            model_path = os.path.join(os.environ['HOME'], '.gazebo', 'models', 'cube_colored', 'cube_red', 'model.sdf')

        with open(model_path, 'r') as f:
            model_xml = f.read()

        model_name = "cube_%d" % int(time.time() * 1000)
        spawn(model_name, model_xml, "/", pose, "world")
        rospy.loginfo("Spawned %s" % model_name)

    except Exception as e:
        rospy.logerr("Failed to spawn cube: %s" % str(e))

def main():
    #rospy.init_node("connect4_cube_spawner")

    print("==== Connect 4 Manual Cube Spawner ====")
    print("Type a column number (0-6) to spawn a red cube.")
    print("Type 9 to spawn a blue cube in a special position.")
    print("Type 'exit' to quit.\n")

    while not rospy.is_shutdown():
        user_input = raw_input("Enter column (0-6 or 9) or 'exit': ")

        if user_input.lower() == 'exit':
            break

        try:
            col = int(user_input)
            if 0 <= col <= 6 or col >= 9:
                spawn_cube(col)
            else:
                print("Please enter a valid column number between 0-6 or 9.")
        except ValueError:
            print("Invalid input. Please enter a number or 'exit'.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
