## Isaac Sim
Isaac Sim is an Nvidia robotics simulator with very powerful GPU-acceleration, parallelization, and photo-realism. It can be used to not only simulate the the UR10e robot but also simulate object collisions and interactions, allowing for simulating complex tasks with the UR10e. Follow these steps to set up the UR10e simulation:


## Setup Instructions:
1. Work through the workstation installation procedure noted [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html). You will want to install the Omniverse Launcher, Cache, Nucleus, and Isaac Sim. Make sure to start a local Nucleus server or the environment will not run properly.
    * Note: you will need to create/login into an NVIDIA account for this step
    * **IMPORTANT**: Make sure you install `2023.1.0-hotfix.1`. Later versions have made API changes.
2. Set up envirnoment variables:
    * The [environment variables](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html) below must be set to their respective paths before running Python scripts.
        * Define an `ISAACSIM_PATH` as the path to the main Isaac Sim directory (e.g. `/home/${USER}/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1`). *Tip*: If struggling to find the path where isaac_sim was installed, go to the Omniverse application. The exact file path will be mentioned under the "Setting" tab for the isaac_sim page.
        * Define a `ISAACSIM_PYTHON_EXE` as the path to `python.sh` in the main Isaac Sim folder (e.g., `/home/${USER}/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/python.sh`)
3. Verify Isaac's installation:
    * Run the example script and verify that an environment with a few cubes spawns.
    ```
    ${ISAACSIM_PYTHON_EXE} ${ISAACSIM_PATH}/standalone_examples/api/omni.isaac.core/add_cubes.py
    ```
4. To use `rclpy` from ROS2 Foxy it is necessary to setup the ROS2 Foxy Isaac Sim workspace as detailed here: https://github.com/isaac-sim/IsaacSim-ros_workspaces.


### Running Experiments:
1. Make sure ROS2 is running.
2. Go to `~/prodapt/src`.
3. Execute `./scripts/run_experiment.sh`. This should open an instance of Isaac Sim with the UR10e arm.
4. Follow the steps in [spacemouse.md](./spacemouse.md) to control the UR10e with the SpaceMouse.


## Environment performance notes
Make sure the local Nucleus server and Cache are running.
* If the simulator is taking a long time (> 20 seconds) to boot up, it is very likely that there is a problem with Nucleus or the Cache.
  * Verify that the Nucleus Server is installed and running
  * Verify that the Cache is installed and running
  * If the problem persists: go to the Settings page for the Nucleus server and "restart all". This should fix performance issues.
