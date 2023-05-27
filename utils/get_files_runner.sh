#!/bin/bash

# List of arguments
args_dir=("Hallway_Full_CCW" "Hallway_Full_CW" "Hallway_Straight_CCW")
args_bag1=("hallway_full_ccw_part_1.bag" "hallway_full_cw_part_1.bag" "hallway_straight_ccw_part_1.bag")
args_bag2=("hallway_full_ccw_part_2.bag" "hallway_full_cw_part_2.bag" "hallway_straight_ccw_part_2.bag")

# Loop through arguments and run Python script with each argument
for i in "${!args_dir[@]}"
do
	echo "$i ${args_dir[i]}"
	echo "$i ${args_bag1[i]}"
	echo "$i ${args_bag2[i]}"
	python3 extract_synced_images.py "${args_dir[i]}" "${args_bag1[i]}" "${args_bag2[i]}"
done

