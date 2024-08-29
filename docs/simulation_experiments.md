## Simulation Experiments

To run simulation experiments of the cube dodging task in Isaac Sim, follow these steps:

1. Ensure Isaac Sim is installed and working by following the steps in [isaacsim.md](./isaacsim.md).
2. Ensure you have generated or downloaded the appropriate diffusion model into the `~/prodapt/checkpoints` folder, along with the appropriate YAML file. The folder structure should be as follows:
    ```
    ├── prodapt
    │   ├── checkpoints
    │   │   ├── <CHECKPOINT_NAME>
    │   │   │   ├── <CHECKPOINT_NAME>.pt
    │   │   │   ├── <CHECKPOINT_NAME>.yaml
    ```
3. In one terminal, run the Isaac Sim software by executing `./scripts/run_isaacsim.bash`.
4. In another terminal, run the experiments. Run one model by executing:
    ```bash
    python3 -m prodapt.main --config-dir=checkpoints/<CHECKPOINT_NAME> --config-name=<CHECKPOINT_NAME>.yaml mode=evaluate
    ```
    * If you wish to change the default parameters of the experiment, you can change them here. For example, to change the max number of control iterations before termination to 2000, append `inference.max_steps=2000`.
    * The mode `evaluate` will simply run `num_inferences` trials. If you wish to only run one trial, you can set `inference.num_inferences=1`, or change the mode to `inference`.
    * If you wish to run many different models, simply modify the shell script [test_all_models.sh](../scripts/test_all_models.sh).

#### How it works
The evaluation works by communicating between the code running Isaac Sim and the code running the diffusion model, with communication done using ROS2 and sockets using the `zmq` Python package.

First, Isaac Sim is set up and is reset to the starting position. To change this position, consult [isaacsim_data_collection.md](./isaacsim_data_collection.md). Then, the Isaac Sim runner will open a socket connection in case the diffusion code sends socket commands. The diffusion code will first load up the requested model. Then it will connect to the socket created by the Isaac Sim runner and send a reset command. The Isaac Sim will be waiting (without blocking) for this command, and will reset the environment when it receives it. Then a trial will start, completing when the diffusion policy code completes (either when `max_steps` iterations have been done or when the trial returns `done=True`). Then the diffusion policy code will send another reset command to the Isaac Sim runner. This will continue until the max number of trials have been completed (this is defined both on the Isaac Sim side and on the diffusion policy side, so the minimum number will cause the experiment to end). 
