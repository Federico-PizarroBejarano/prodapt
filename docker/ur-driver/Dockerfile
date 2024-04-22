FROM osrf/ros:humble-desktop-full

RUN apt-get update \
        && apt-get -y install \
        ros-${ROS_DISTRO}-ur \
        python3-rosdep \
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN rosdep update

ENV ROBOT_IP=192.168.56.101
ENV UR_TYPE="ur10e"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
