#!/bin/bash
.  /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false
