# prodapt
Proprioceptive Adaptation for Autonomous Manipulation Tasks

## References
The initial work in this repo is evolving from cleaning up sections of code from: https://github.com/real-stanford/diffusion_policy

## Debugging
A list of common problems that may be encountered when first setting up the repo

---
Error:
```
AssertionError: Cannot find installation of real FFmpeg (which comes with ffprobe)
```
Problem:
FFmpeg has likely not been installed or updated on your host

Solution:
```
sudo apt-get install ffmpeg
```
---
