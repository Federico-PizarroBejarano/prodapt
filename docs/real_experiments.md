## Real Experiments

To run real experiments of the cube dodging task, follow these steps:

1. Switch to the `real_experiments` branch of the repository.
2. Ensure that your host computer has ROS2 and is connected to the UR10e via Ethernet. Execute the installation script [install.bash](../scripts/install.bash) to make sure you have all necessary packages.
3. Power on the UR10e.
    1. Ensure that you can ping the UR10e's IP address from the host computer. This IP address is found by going to the menu icon in the top right of the pendant and clicking "About".
    2. Ensure that the External Control IP Address on the pendant is the correct IP address for your host computer. You can see this by going to the Installation tab on the pendant and clcking "URCaps" and then "External Control". Your host IP address is set by connecting the robot to the computer via Ethernet, then setting up the Ethernet to be "Manual" on IPv4, setting the desired IP address, setting the Netmask to ""255.255.255.0", and setting the DNS to Automatic. Then turn off and turn on the connection.
4. Run the driver on the host computer:
    1. Run the driver by executing
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=${ROBOT_IP} launch_rviz:=false initial_joint_controller:=forward_velocity_controller
        ```
    2. Confirm this is working by, in another tab, running `ros2 topic list` and ensuring the UR10 topics are being published.
    3. In another tab, switch the ROS2 controller to velocity control by executing
        ```bash
        ros2 control switch_controllers --activate forward_velocity_controller --deactivate [scaled_joint_trajectory_controller forward_position_controller]
        ```
5. Put the UR10e in External Control mode. Simply go to the Program tab on the pendant, click URCaps, and then External Control. Then hit the play button in the lower right corner.
6. Run the diffusion controller the same as you would in simulation, by following [simulation_experiments.md](./simulation_experiments.md).

#### Nuances of Real Experiments
1. **Velocity Control**: In the simulation experiments, position control was used. This works well in simulation, but it was found to be unreasonably jerky when executing on the real robot. Thus, we switched to velocity control. The diffusion policy still outputs Cartesian position commands, but the lower level ROS2 controller that executes the commands now executes them as joint-velocity commands rather than joint-position commands, using the `/forward_velocity_controller/commands` topic rather than the `/scaled_joint_trajectory_controller/joint_trajectory` topic.
2. To prevent jerkiness and safeguard the UR10e arm, limits were placed on the joint speeds and the joint accelerations. This can be found in [joints_publisher.py](../src/prodapt/envs/ur10_ros_interface/joints_publisher.py).
3. To prevent very high torque being applied and triggering the Protective Stop behaviour of the pendant, the robot was prevented from moving in the direction of high torque. This can be found in the `post_process_action` function in [diffusion_policy.py](../src/prodapt/diffusion_policy.py).
4. The $x$ and $y$ directions were transformed according to the `real_exp_transform` function in [rotation_utils.py](../src/prodapt/utils/rotation_utils.py). This was to ensure a consistent setup betwen our simulated experiments and our real testbed, which was oriented differently than simulation.
