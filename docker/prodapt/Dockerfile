FROM osrf/ros:humble-desktop-full

WORKDIR /root
COPY ./pyproject.toml /root/pyproject.toml
COPY ./docker/prodapt/scripts/install_dependencies.bash /root/scripts/install_dependencies.bash
COPY ./docker/prodapt/scripts/.bashrc /root/.bashrc
RUN sh /root/scripts/install_dependencies.bash && apt-get clean -y && rm -rf /var/lib/apt/lists/*
RUN rosdep update

ENV ROBOT_IP=192.168.56.20
ENV UR_TYPE="ur10e"

# Copy entrypoint script
COPY ./docker/prodapt/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-l", "/entrypoint.sh" ]
