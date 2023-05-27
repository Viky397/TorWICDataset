#!/bin/bash

# List of arguments
args_dir=("Hallway_Full_CCW" "Hallway_Full_CW" "Hallway_Straight_CCW")
args_cam=("left" "right")

# Loop through arguments and run Python script with each argument
for arg_dir in "${args_dir[@]}"
do
	for arg_cam in "${args_cam[@]}"
	do
		python3 cpr_onnx_seg_inference.py "$arg_dir" "$arg_cam"
	done
done

