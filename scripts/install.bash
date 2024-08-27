#!/bin/bash
sudo apt-get update

# Install Python packages
apt-get -y install python3 python3-pip
python3 -m pip install --upgrade pip
python3 -m pip install -e . --extra-index-url https://download.pytorch.org/whl/cu118 --timeout=1000
pip3 install transforms3d

# Install miscellaneous packages
sudo apt-get -y install unzip
sudo apt-get -y install ffmpeg

# Install Spacenav packages
sudo apt-get -y install spacenavd
sudo apt-get -y install ros-${ROS_DISTRO}-spacenav
sudo apt-get -y install ros-${ROS_DISTRO}-tf-transformations

# ROS2 Control
sudo apt-get update
sudo apt-get -y install ros-${ROS_DISTRO}-ros2-control
sudo apt-get -y install ros-${ROS_DISTRO}-ros2-controllers

# Install UR drivers and rosdep
apt-get -y install ros-${ROS_DISTRO}-ur
sudo apt-get -y install python3-rosdep

# Clean
sudo apt-get autoremove -y
sudo apt-get autoclean -y
