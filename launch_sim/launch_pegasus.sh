#!/bin/bash

# Set the conda environment
export ISSAC_CONDA_ENV="env_isaacsim"

# Isaac Sim root directory  
export ISAACSIM_PATH="${HOME}/isaacsim"

# Activate the conda environment
source ~/miniconda3/etc/profile.d/conda.sh 
conda activate ${ISSAC_CONDA_ENV}

# Source the conda environment
source ${ISAACSIM_PATH}/setup_conda_env.sh

# Execute the Python script
echo "Running Python script..."

python3 1_px4_single_vehicle.py

