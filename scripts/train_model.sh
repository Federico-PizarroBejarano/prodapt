#!/bin/bash
#SBATCH -J train_model                      # Job name
#SBATCH -o ./slurm/train_model_%j.out       # Name of job output file
#SBATCH -p gpu                              # Queue (partition) name
#SBATCH -N 1                                # Total # of nodes per instance
#SBATCH -n 8                                # Total # of cores
#SBATCH -G 1                                # Total # of GPUs
#SBATCH --mem=40G                           # Memory (RAM) requested
#SBATCH -t 12:00:00                         # Run time (hh:mm:ss)
#SBATCH --mail-type=none                    # Send email at begin and end of job
#SBATCH --mail-user=bejarano@jpl.nasa.gov

cd /scratch/prodapt/prodapt
python -m prodapt.main --config-name=ur10_cube.yaml mode=train
