## Controlling the UR10e with a 3D Mouse

It is possible to control robots using a 3D mouse such as the 3DConnexion SpaceMouse. This allows for controlling the end-effector's position and orientation with one hand.

To accomplish this, the `spacenav` ROS package is used (https://index.ros.org/p/spacenav/) which converts the signal from the 3Dconnexion SpaceMouse into various ROS2 messages, including `/spacenav/twist (geometry_msgs/msg/Twist)`. The custom ROS2 package [spacenav_to_movel](../src/spacenav_to_movel/spacenav_to_movel/converter.py) is used to listen to the `/spacenav/twist` topic and convert those messages to MoveL commands (a linear move command used by the Universal Robots scripting language, URScript) which is sent to the robot (real or simulated) via the `/urscript_interface/script_command` topic.


### Steps
1. Turn on the UR robot and run the `ur-robot-driver`. Make sure the robot is externally controllable, such as by following the steps in [docker.md](./docker.md).
2. Ensure you have the SpaceMouse and `tf-transformations` packages installed:
    ```bash
    sudo apt-get install spacenavd
    sudo apt-get install ros-foxy-spacenav
    sudo apt-get install ros-foxy-tf-transformations
    ```
3. Plug in the SpaceMouse
4. Run the `spacenav` node by running:
    ```bash
    ros2 run spacenav spacenav_node
    ```
    And check that it is publishing by running
    ```bash
    ros2 topic echo /spacenav/twist
    ```
    You should see the topic being published to rapidly, and it should react to manipulating the SpaceMouse.
5. Build the `spacenav_to_movel` package (only needed to be done once, and after any code change) by running
    ```bash
    cd ~/prodapt
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select spacenav_to_movel
    ```
6. Run the `spacenav_to_movel` package by running
    ```bash
    cd ~/prodapt
    source install/setup.bash
    ros2 run spacenav_to_movel converter
    ```
    Now you should be able to control the robot with the SpaceMouse by moving the SpaceMouse in the desired direction and orientation (i.e., not touching the SpaceMouse should cause the robot to stay still).


### Troubleshooting
If you get the error
```
tf2.ConnectivityException: Could not find a connection between 'base' and 'tool0' because they are not part of the same tree.Tf has two or more unconnected trees.
```
or another `tf` error after running the `spacenav_to_movel` package, simply kill the `ur-driver` process, confirm the robot (simulated or real) is powered on and started, then restart the `ur-driver`. You can check the `tf` frames are correct and connected by creating a PDF of the frames using
```bash
ros2 run tf2_tools view_frames.py
```
and view the PDF with
```bash
xdg-open frames.pdf
```
