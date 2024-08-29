## UR10 Data Collection

### Simulation
To collect data for the UR10 in URSim, follow this procedure:
1. Setup URSim as detailed in [ursim.md](./ursim.md). Go to the **Move** tab in URSim and change the Feature to **Base**. This makes visualization easier.
2. Setup the `spacenav_converter` as detailed in [spacemouse.md](./spacemouse.md).
   1. Confirm that you can control the UR10 in URSim using the SpaceMouse.
   2. Kill the terminal running the `spacenav_converter`.
3. Reset the UR10 to the starting position using
   ```bash
   ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
    "def my_prog():

    movej(p[0.6, 0, 0.4, 3.14, 0, 0], a=1.2, v=0.25, r=0)

    end"}'
    ```
4. Restart the `spacenav_converter` ROS node.
5. Start data collection:
   1. In a separate terminal, navigate to `~/prodapt/data/ur10/trajectories`
   2. Execute
        ```bash
        ros2 bag record -o traj{TRAJ_NUM} /joint_states /urscript_interface/script_command
        ```
      where ***TRAJ_NUM*** is the number of the trajectory you are saving. If using joint control, record `/scaled_joint_trajectory_controller/joint_trajectory` instead of `/urscript_interface/script_command`.
   3. Control the UR10 using the SpaceMouse to the desired trajectory.
6. Wrapping up:
   1. When the trajectory is complete, kill the ROS bag recording node.
   2. Kill the `spacenav_converter` node.
7. Repeat steps 3-6 for additional trajectories.
8. The rosbag data can now be processed into a `zarr` dataset using [process_ursim_data.py](../src/prodapt/dataset/process_ursim_data.py).
