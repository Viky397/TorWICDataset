#!/bin/bash

# List of arguments
args_dir=("Hallway_Full_CCW" "Hallway_Full_CW" "Hallway_Straight_CCW")

# Loop through arguments and run Python script with each argument
for arg in "${args_dir[@]}"
do
    python3 get_gt_pose.py "$arg"
done

