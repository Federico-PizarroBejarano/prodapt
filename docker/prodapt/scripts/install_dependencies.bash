#!/bin/bash
apt-get update

# Install Python packages
apt-get -y install python3 python3-pip
python3 -m pip install --upgrade pip
python3 -m pip install -e . --extra-index-url https://download.pytorch.org/whl/cu118 --timeout=1000
pip3 install transforms3d

# Install miscellaneous packages
apt-get -y install unzip
apt-get -y install ffmpeg

# Install Spacenav packages
apt-get -y install spacenavd
apt-get -y install ros-${ROS_DISTRO}-spacenav
apt-get -y install ros-${ROS_DISTRO}-tf-transformations

# ROS2 Control
apt-get update
apt-get -y install ros-${ROS_DISTRO}-ros2-control
apt-get -y install ros-${ROS_DISTRO}-ros2-controllers

# Install UR drivers and rosdep
apt-get -y install ros-${ROS_DISTRO}-ur
apt-get -y install python3-rosdep

apt-get autoremove -y
apt-get clean -y
