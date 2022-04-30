import os
import sys
import numpy as np
from tqdm import tqdm
import rosbag

from pose import Pose
from imu import IMU
from odom import Odom
from laser import Laserscan
from rgb import RGBImage
from depth import DepthImage
from mask import MaskImage
from calibration import getStaticTFMsg

from data_loader import DataLoader

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        raise RuntimeError("Need at least one trajectory ID")
    sequences = sys.argv[1:]
    num_seq = len(sequences)
    print("Converting sequence(s):", sequences)

    # Load the desired trajectories
    datasets = []
    print("Loading sequences...")
    for i in tqdm(range(num_seq)):
        data = DataLoader(sequences[i])
        datasets.append(data)
    
    # Calculate the time gap between consecutive trajectories
    if num_seq > 1:
        for i in range(1,num_seq):
            dt = datasets[i].start_time - datasets[i-1].end_time
            datasets[i].minusTimeOffset(dt)

    # Write all data to an output bag
    traj_start_time = datasets[0].start_time
    bag_name = './outputs/merged_' + '_'.join(sequences) + '.bag'
    if os.path.exists(bag_name):
        os.remove(bag_name)
    output_bag = rosbag.Bag(bag_name, 'w')
    print("Writing to bag", bag_name)
    for i in tqdm(range(num_seq)):
        # Write poses and TF
        for pose in datasets[i].poses:
            pose_message = pose.toROSMsg()
            tf_msg = pose.toROSTF()
            static_tf_msg = getStaticTFMsg(pose.id, pose.time)
            output_bag.write(pose.topic, pose_message, pose_message.header.stamp)
            output_bag.write('/tf', tf_msg, pose_message.header.stamp)
            output_bag.write('/tf_static', static_tf_msg, pose_message.header.stamp)
            
        # Write odoms
        for odom in datasets[i].odoms:
            odom_message = odom.toROSMsg()
            output_bag.write(odom.topic, odom_message, odom_message.header.stamp)

        # Write IMUs
        for imu in datasets[i].imus:
            imu_message = imu.toROSMsg()
            output_bag.write(imu.topic, imu_message, imu_message.header.stamp)

        # Write lasers
        for laser in datasets[i].lasers:
            laser_message = laser.toROSMsg()
            output_bag.write(laser.topic, laser_message, laser_message.header.stamp)

        # Write images and caminfo
        for rgb in datasets[i].images_rgbs:
            rgb_message = rgb.toROSMsg()
            rbg_info = rgb.getCameraInfo()
            output_bag.write(rgb.topic+'image_raw', rgb_message, rgb_message.header.stamp)
            output_bag.write(rgb.topic+'camera_info', rbg_info, rgb_message.header.stamp)
        for depth in datasets[i].images_depths:
            depth_message = depth.toROSMsg()
            depth_info = depth.getCameraInfo()
            output_bag.write(depth.topic+'image_raw', depth_message, depth_message.header.stamp)
            output_bag.write(depth.topic+'camera_info', depth_info, depth_message.header.stamp)
        for mask in datasets[i].images_masks:
            mask_message = mask.toROSMsg()
            mask_info = mask.getCameraInfo()
            output_bag.write(mask.topic+'image_raw', mask_message, mask_message.header.stamp)
            output_bag.write(mask.topic+'camera_info', mask_info, mask_message.header.stamp)

    output_bag.close()