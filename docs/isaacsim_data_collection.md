## Isaac Sim Data Collection

### Simulation
To collect data for the UR10 in Isaac Sim, follow this procedure:
1. Setup Isaac Sim as detailed in [isaacsim.md](./isaacsim.md).
2. Setup the `spacenav_converter` as detailed in [spacemouse.md](./spacemouse.md).
   1. Confirm that you can control the UR10 in Isaac Sim using the SpaceMouse.
3. The UR10 in Isaac Sim starting pose is controlled by `positions` in [ur10e.py](../src/simulator_isaac/ur10e.py). If you wish to change the starting pose, change it here.
4. The code controlling the Isaac Sim simulation, [simulator.py](../src/simulator_isaac/simulator.py), is geared towards communication with the main diffusion code, [diffusion_policy.py](../src/prodapt/diffusion_policy.py). To allow the simulation to reset at desired points (to avoid needing to reset it yourself), remove the socket communication code from [simulator.py](../src/simulator_isaac/simulator.py) (which uses the `zmq` package) and instead add your own reset conditions. The trajectories can be distinguished in the rosbag data later by the sudden change in position due to the reset.
5. Start data collection:
   1. In a separate terminal, navigate to `~/prodapt/data/ur10`
   2. Execute
        ```bash
        ros2 bag record -o {DATASET_NAME} /joint_states /scaled_joint_trajectory_controller/joint_trajectory
        ```
      If recording force-torque data, ensure you additionally record `/force_torque_sensor_broadcaster/wrench`.
   3. Control the UR10 using the SpaceMouse to the desired trajectory.
6. Wrapping up:
   1. When the trajectory is complete, kill the ROS bag recording node.
   2. Kill the `spacenav_converter` node.
7. The rosbag data can now be processed into a `zarr` dataset using [process_isaacsim_data.py](../src/prodapt/dataset/process_isaacsim_data.py).
