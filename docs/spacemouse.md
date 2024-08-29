## Controlling the UR10e with a 3D Mouse

It is possible to control robots using a 3D mouse such as the 3DConnexion SpaceMouse. This allows for controlling the end-effector's position and orientation with one hand. The following steps can be run natively or on the Docker container.

To accomplish this, the `spacenav` ROS package is used (https://index.ros.org/p/spacenav/) which converts the signal from the 3DConnexion SpaceMouse into various ROS2 messages, including `/spacenav/twist (geometry_msgs/msg/Twist)`. The custom ROS2 package `spacenav_converter` is used to listen to the `/spacenav/twist` topic. This twist is turned into a desired pose offset, and used to calculate the next desired pose. The desired pose is then sent as a command to the UR10 via (currently) two possible interfaces:
- MoveL commands (a linear move command used by the Universal Robots scripting language, URScript) which is sent to the robot (real or simulated via URSim) via the `/urscript_interface/script_command` topic.
- Joint positions, calculated using the inverse kinematics in [kinematics_utils.py](../src/spacenav_converter/spacenav_to_movel/kinematics_utils.py), sent to the robot via the `/scaled_joint_trajectory_controller/joint_trajectory` topic (if using URsim or the real robot) or the `/joint_command` topic (if simulating the robot in Isaac Sim).
    - **Note** The real robot should be controlled using a velocity control interface, outlined in [real_experiments.md](./real_experiments.md).

### Installation
1. Ensure you have the SpaceMouse and `tf-transformations` packages installed (already done in the ProDapt Docker container):
    ```bash
    sudo apt-get install spacenavd
    sudo apt-get install ros-${ROS_DISTRO}-spacenav
    sudo apt-get install ros-${ROS_DISTRO}-tf-transformations
    ```
2. Build the `spacenav_converter` package (only needed to be done once, and after any code change) by running
    ```bash
    cd ~/prodapt
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
    colcon build --packages-select spacenav_converter
    ```

### Steps
1. (If using URSim) Turn on the UR robot (or URSim) and run the `ur-robot-driver` (either locally, in it's own container, or in the ProDapt container). Make sure the robot is externally controllable, such as by following the steps in [ursim.md](./ursim.md).
2. (If using Isaac Sim) Start Isaac Sim as detailed in [isaacsim.md](./isaacsim.md). Ensure the simulation is started and not paused.
3. Plug in the SpaceMouse.
4. Run the `spacenav` node by running:
    ```bash
    ros2 run spacenav spacenav_node
    ```
    And check that it is publishing by running
    ```bash
    ros2 topic echo /spacenav/twist
    ```
    You should see the topic being published to rapidly, and it should react to manipulating the SpaceMouse.
5. Run the `spacenav_converter` package by running
    ```bash
    cd ~/prodapt
    source install/setup.bash

    # If using URSim
    ros2 run spacenav_converter converter_movel  # or 'converter_joints' to control via joint positions
    # If using Isaac Sim
    ros2 run spacenav_converter converter_joints --ros-args -p interface:=isaacsim
    ```
    Now you should be able to control the robot with the SpaceMouse by moving the SpaceMouse in the desired direction and orientation (i.e., not touching the SpaceMouse should cause the robot to stay still).


### Troubleshooting
If you get the error
```
tf2.ConnectivityException: Could not find a connection between 'base' and 'tool0' because they are not part of the same tree.Tf has two or more unconnected trees.
```
or another `tf` error after running the `spacenav_converter` package, simply kill the `ur-driver` process, confirm the robot (simulated or real) is powered on and started, then restart the `ur-robot-driver`. You can check the `tf` frames are correct and connected by creating a PDF of the frames using
```bash
ros2 run tf2_tools view_frames.py
```
and view the PDF with
```bash
xdg-open frames.pdf
```

---

If the robot is not moving when using the joint commands on URSim, first confirm that the robot is externally controllable by following [ursim.md](./ursim.md). If it is, confirm that the `/scaled_joint_trajectory_controller/joint_trajectory` has a subscriber by running
```bash
ros2 topic info /scaled_joint_trajectory_controller/joint_trajectory
```
If there are no subscribers, kill the URSim container and the `ur-driver` and restart them until the topic appears in `ros2 topic list` and has a subscriber.

---

If running `ros2 run spacenav spacenav_node` is resulting in an error or `ros2 topic echo /spacenav/twist` is always returning 0 for every value, check the following:
1. Ensure the Spacemouse is connected
2. Ensure that the host computer has installed the `spacenav` packages by running:
    ```bash
    sudo apt-get install spacenavd
    service spacenavd restart
    ```
3. Ensure that the socket is properly formatted by running `ls -l /var/run/ | grep 'spnav.sock'`. If nothing shows up, something is wrong and the socket is not being created. If the socket appears, ensure the first section reads `srwxrwxrwx`, and does not start with a `d` (for directory). If it does, delete it and rerun the previous step and ensure a proper socket is created.
