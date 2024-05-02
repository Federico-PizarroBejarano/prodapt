## Using Docker

There are two Docker containers described in `prodapt/docker`, one for the URSim Simulator and one for the ROS2 Humble driver for Universal Robots ([ur-driver](../docker/ur-driver/Dockerfile)). This Docker setup is inspired by https://github.com/maffettone/erobs/.


### Running the Docker Containers
1. Build the two images.
    ```bash
    cd docker
    docker pull universalrobots/ursim_e-series
    docker build -t ur-driver:latest ./ur-driver
    ```
2. Start URSim. Run
    ```bash
    docker compose up ursim
    ```
    Open VNC client at http://localhost:6080/vnc.html and click Connect.
    Turn on and start the robot.
3. Start the ur-driver. In a new terminal, run
    ```bash
    docker compose up ur-driver
    ```


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

    set_digital_out(1, True)

    movej(p[0.6, 0, 0.4, 3.14, 0, 0], a=1.2, v=0.25, r=0)

    end"}'
    ```
    You should see the robot move the end-effector to position [-0.6, -0.3, 0.4] with orientation [3.14, 0, 0].
