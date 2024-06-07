#!/bin/bash
.  /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/prodapt
python3 -m pip install -e .
service spacenavd restart
/bin/bash
