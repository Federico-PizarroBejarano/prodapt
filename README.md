# ProDapt
Proprioceptive Adaptation for Autonomous Manipulation Tasks


## References
The initial work in this repo evolved from cleaning up sections of code from: https://github.com/real-stanford/diffusion_policy.


## Installation
This repository can be used either natively or using Docker. Regardless, we assume the directory is cloned in your home directory.

#### Direct Installation
This repository has been tested using ROS2 Foxy installed on Ubuntu 20.04, using Python 3.8, with relevant packages installed via:
```bash
./scripts/install.bash
```
However, everything should also work for ROS2 Humble on Ubuntu 22.04 and Python 3.10.

#### Docker
The docker image for the project can be built using
```bash
cd ~/prodapt
docker build -t prodapt:latest -f ./docker/prodapt/Dockerfile .
```
and then run via
```bash
cd ~/prodapt/docker
docker compose up prodapt
```
This container is for Ubuntu 22.04 to run ROS2 Humble. It has all the necessary packages and Python modules installed. Additionally, it loads the prodapt repository as a volume so work can be done directly from the Docker container.

Open the container using VSCode's Dev Containers extension, using the command `Dev Containers: Attach to Running Container`. Then, both running and developing this repository can be done from inside the container.

**Note:** If you wish to use the Spacemouse (see [spacemouse.md](./docs/spacemouse.md)) to control the UR10e robot (either in simulation or real life), you will need to run docker from a Linux computer. Before building the docker container for the first time run:
```bash
sudo apt-get install spacenavd
service spacenavd restart
```


## Running Push-T Example
Download the trained weights for the push-T example by running
```bash
./scripts/download_push_t_weights.bash
```
which should create the folder `prodapt/checkpoints` and add the file `pusht_state_100ep.ckpt`. Then, run the experiment by running
```bash
python3 -m prodapt.main --config-name=push_t.yaml mode=inference
```
which should run an evaluation that has a final score of approximately 0.95. A video of the experiment can be found in the `prodapt/output` folder, in the latest experiment.


## Training Push-T Example
Download the training data for the push-T example by running
```bash
./scripts/download_push_t_data.bash
```
which should create the folder `prodapt/data/push_t_data.zarr` folder and populate it with data. Then, run the training by running
```bash
python3 -m prodapt.main --config-name=push_t.yaml mode=train
```
This will begin training (100 epochs), which should take ~10min. Then, evaluate the policy by running
```bash
python3 -m prodapt.main --config-name=push_t.yaml mode=inference inference.checkpoint_path=./checkpoints/diffusion_policy.pt
```
which could return a score of approximately 0.95.


## Debugging
A list of common problems that may be encountered when first setting up the repo

---
Error:
```
AssertionError: Cannot find installation of real FFmpeg (which comes with ffprobe)
```
Problem:
FFmpeg has likely not been installed or updated on your host.

Solution:
```
sudo apt-get install ffmpeg
```
---
