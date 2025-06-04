# 🤖 Connect 4 Robot Simulation (Gazebo + ROS + AI)

This project is a 3D simulation of a robot playing the Connect 4 game autonomously using artificial intelligence.

It uses **ROS1 (Kinetic)**, **Gazebo 7**, **MoveIt**, **OpenCV**, and a simulated **Fetch robot** to detect colored cubes, plan grasps, and make strategic moves against a human player in a turn-based Connect 4 game.

---

## 🎥 Demo

> <img src="video/video.mp4" align="middle"> 

## 🛠️ Installation

> ✅ Recommended system: **Ubuntu 16.04 LTS**

### Required Software

- ROS Kinetic
- Gazebo 7
- Python 2.7
- MoveIt
- OpenCV

### Required Dependencies

# Update your system
sudo apt update
sudo apt upgrade

# ROS base dependencies
sudo apt install -y \
  ros-kinetic-roscpp \
  ros-kinetic-rospy \
  ros-kinetic-std-msgs \
  ros-kinetic-sensor-msgs

# Gazebo integration
sudo apt install -y \
  ros-kinetic-gazebo-ros \
  ros-kinetic-gazebo-plugins

# Control-related packages
sudo apt install -y \
  ros-kinetic-control-toolbox \
  ros-kinetic-robot-controllers \
  ros-kinetic-robot-controllers-interface

# General utilities
sudo apt install -y \
  libboost-all-dev \
  python-opencv
