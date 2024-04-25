# prodapt
Proprioceptive Adaptation for Autonomous Manipulation Tasks

## References
The initial work in this repo is evolving from cleaning up sections of code from: https://github.com/real-stanford/diffusion_policy

## Installation
Create a conda environment:
```bash
conda create --name prodapt python=3.10
conda activate prodapt
```
Then install all the relevant packages:
```bash
./scripts/install.bash
```

## Running Push-T Example
Download the trained weights for the push-T example by running
```bash
./scripts/download_push_t_weights.bash
```
which should create the folder `prodapt/checkpoints` and add the file `pusht_state_100ep.ckpt`. Then, run the experiment by running
```bash
python -m prodapt.main main.mode=inference
```
which should run an evaluation that has a final score of 1.0. A video of the experiment can be found in the `prodapt/output` folder, in the latest experiment.

## Training Push-T Example
Download the training data for the push-T example by running
```bash
./scripts/download_push_t_data.bash
```
which should create the folder `prodapt/data/push_t_data.zarr` folder and populate it with data. Then, run the training by running
```bash
python -m prodapt.main main.mode=train
```
This will begin training (100 epochs), which should take ~10min. Then, evaluate the policy by running
```bash
python -m prodapt.main main.mode=inference main.checkpoint=./checkpoints/diffusion_policy.pt
```
which could return a score of approximately 0.947.

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
