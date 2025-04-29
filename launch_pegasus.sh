#!/bin/bash

# Set the conda environment
export ISAAC_CONDA_ENV="isaac_env"

# Isaac Sim root directory  
export ISAACSIM_PATH="$HOME/isaacsim"

# Activate the conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate $ISAAC_CONDA_ENV

# Source the conda environment
source $ISAACSIM_PATH/setup_conda_env.sh

# Initialize the Micro XRCE-DDS Agent
echo "Initializing Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &

# Initialize the lidar filter
# ros2 run pegasus_sim_env lidar_filter_node &

# Execute the Python script
echo "Running Python script..."
python src/pegasus_sim_env/environment/pegasus_environment.py

# Deactivate the conda environment
conda deactivate

pkill MicroXRCEAgent

# End of file



