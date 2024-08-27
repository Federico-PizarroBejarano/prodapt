source ~/IsaacSim-ros_workspaces/build_ws/foxy/foxy_ws/install/setup.bash
source ~/IsaacSim-ros_workspaces/build_ws/foxy/isaac_sim_ros_ws/install/setup.bash
cd ~/prodapt/src
$ISAACSIM_PYTHON_EXE -m simulator_isaac.main $1
