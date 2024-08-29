## URSim

URSim is the dedicated simulator of Universal Robots, which accurately simulates not only robot dynamics but the interface and drivers used to control their robots. This allows one to very accurately determine if software and algorithms will work as expected on the real robot by first testing in URSim. To run URSim, there is a Docker container in `prodapt/docker`. Additionally, to run the ROS2 driver to connect to URSim (and the real robot), there is a dedicated container for just the driver ([ur-driver](../docker/ur-driver/Dockerfile)), as well as a larger container that contains everything needed to develop this project ([prodapt](../docker/prodapt/Dockerfile)), including the driver.


### Running the Docker Containers
1. Build the two images.
    ```bash
    cd ~/prodapt/docker
    docker build -t ursim:latest ./ursim
    docker build -t ur-driver:latest ./ur-driver
    ```
    **Note:** The `ur-driver` docker container is a light-weight version of the `prodapt` container. If you wish to build and use the `prodapt` container instead, follow the instructions in [README.md](../README.md)
2. Start URSim. Run
    ```bash
    docker compose up ursim
    ```
    Open VNC client at http://localhost:6080/vnc.html and click Connect.
    Turn on and start the robot.
3. Start the UR driver. In a new terminal, run
    ```bash
    docker compose up ur-driver
    ```
    **Note:** If you already are using the `prodapt` container you do not need to start the `ur-driver` container. Simply open a shell in your container and run the command
    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false
    ```

#### Enabling External Control
When externally controlling the robot (simulated or real), it is usually necessary to enable this in the software. If sending commands via URScript (via the `/urscript_interface/script_command` ROS topic) this is not necessary, but it is necessary for sending ROS control commands such as joint position commands on `scaled_joint_trajectory_controller/joint_trajectory`. To enable external control, follow these steps:
1. Open the "Programs", and on the left side click on "URCaps", then on "External Control". This should add a new line in the program that says "Control by 192.168.56.102".
2. Run the program by clicking on the play button in the bottom right corner of the screen, and click "Play from the beginning". This should allow the robot to be externally controlled. This will need to be rerun if the robot stops running the program.


### Confirming it is Working
1. Once both containers are running, it should be possible to see the ROS topics. In a new terminal, run:
   ```bash
   ros2 topic list -t
   ```
   and you should see many ROS topics, including `/urscript_interface/script_command [std_msgs/msg/String]`
2. Go to the Program page and click on Graphics tab. Then, run
   ```bash
    ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
    "def my_prog():

    movej(p[0.6, 0, 0.4, 3.14, 0, 0], a=1.2, v=0.25, r=0)

    end"}'
    ```
    You should see the robot move the end-effector to position [-0.6, 0.0, 0.4] with orientation [3.14, 0, 0].
