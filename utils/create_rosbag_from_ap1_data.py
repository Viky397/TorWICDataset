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

from data_loader_ap1 import DataLoader

if __name__ == '__main__':

    folder = "/home/jqian/Downloads/demo-longloopreverse/"

    data = DataLoader(folder)
    times = np.loadtxt(folder + "/times.txt")
    N = len(times)

    # Write all data to an output bag
    traj_start_time = times[0]
    bag_name = './outputs/out.bag'
    if os.path.exists(bag_name):
        os.remove(bag_name)

    output_bag = rosbag.Bag(bag_name, 'w')
    print("Writing to bag", bag_name)



    # Write poses and TF
    for pose in data.poses:
        pose_message = pose.toROSMsg()
        tf_msg = pose.toROSTF()
        static_tf_msg = getStaticTFMsg(pose.id, pose.time)
        output_bag.write(pose.topic, pose_message, pose_message.header.stamp)
        output_bag.write('/tf', tf_msg, pose_message.header.stamp)
        output_bag.write('/tf_static', static_tf_msg, pose_message.header.stamp)

    # Write images and caminfo
    for rgb in data.images_rgbs:
        rgb_message = rgb.toROSMsg()
        rbg_info = rgb.getCameraInfo()
        output_bag.write(rgb.topic+'image_raw', rgb_message, rgb_message.header.stamp)
        output_bag.write(rgb.topic+'camera_info', rbg_info, rgb_message.header.stamp)
    for depth in data.images_depths:
        depth_message = depth.toROSMsg()
        depth_info = depth.getCameraInfo()
        output_bag.write(depth.topic+'image_raw', depth_message, depth_message.header.stamp)
        output_bag.write(depth.topic+'camera_info', depth_info, depth_message.header.stamp)
    for mask in data.images_masks:
        mask_message = mask.toROSMsg()
        mask_info = mask.getCameraInfo()
        output_bag.write(mask.topic+'image_raw', mask_message, mask_message.header.stamp)
        output_bag.write(mask.topic+'camera_info', mask_info, mask_message.header.stamp)

    output_bag.close()