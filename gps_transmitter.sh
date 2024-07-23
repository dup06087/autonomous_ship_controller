#!/bin/bash
#source ~/mambaforge/etc/profile.d/conda.sh
source ~/anaconda3/etc/profile.d/conda.sh
conda activate main_env

python3 ./24.07.17_local_planner/gps_transmitter/gps_transmitter.py
