#!/bin/bash
.  /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.20 launch_rviz:=false initial_joint_controller:=forward_velocity_controller
ros2 control switch_controllers --activate forward_velocity_controller --deactivate [scaled_joint_trajectory_controller forward_position_controller]
