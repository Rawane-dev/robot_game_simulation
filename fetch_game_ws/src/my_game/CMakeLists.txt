cmake_minimum_required(VERSION 3.0.2)
project(my_game)

# Trouver les packages ROS nécessaires
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
