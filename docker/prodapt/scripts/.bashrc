export HYDRA_FULL_ERROR=1

### ROS 2 ###
source /opt/ros/${ROS_DISTRO}/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
