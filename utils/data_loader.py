import os
import sys
import cv2
import numpy as np
from tqdm import tqdm

from sort_nicely import sort_nicely
from pose import Pose
from imu import IMU
from odom import Odom
from laser import Laserscan
from rgb import RGBImage
from depth import DepthImage
from mask import MaskImage

class DataLoader:
    def __init__(self, sequence_name):
        self.folder = "./Scenario_" + sequence_name
        self.rgb_folder = self.folder + "/RGB/"
        self.depth_folder = self.folder + "/Depth/"
        self.laser_folder = self.folder + "/LaserScans/"
        self.mask_folder = self.folder + "/Segmentation/"
        self.pose_file = self.folder + "/poses.txt"
        self.odom_file = self.folder + "/odom.txt"
        self.imu_file = self.folder + "/imu.txt"

        self.images_size = 0
        self.images_rgbs = []
        self.images_depths = []
        self.images_masks = []

        self.poses_size = 0
        self.poses_time = []
        self.poses = []

        self.lasers_size = 0
        self.lasers = []

        self.imus_size = 0
        self.imus = []

        self.odoms_size = 0
        self.odoms = []

        self.loadPoses()
        self.loadOdoms()
        self.loadIMUs()
        self.loadLasers()
        self.loadImages()

    def loadPoses(self):
        pose_txt = np.loadtxt(self.pose_file, skiprows=0)
        self.poses_size = sum(1 for line in pose_txt)
        print("Loading Poses")
        for i in tqdm(range(self.poses_size)):
            pose = Pose(i, pose_txt[i][1:])
            self.poses_time.append(pose.time)
            self.poses.append(pose)

    def loadIMUs(self):
        imu_txt = np.loadtxt(self.imu_file, skiprows=0)
        self.imus_size = sum(1 for line in imu_txt)
        print("Loading IMUs")
        for i in tqdm(range(self.imus_size)):
            imu = IMU(i, imu_txt[i][1:])
            self.imus.append(imu)

    def loadOdoms(self):
        odom_txt = np.loadtxt(self.odom_file, skiprows=0)
        self.odoms_size = sum(1 for line in odom_txt)
        print("Loading Odoms")
        for i in tqdm(range(self.odoms_size)):
            odom = Odom(i, odom_txt[i][1:])
            self.odoms.append(odom)

    def loadOdoms(self):
        odom_txt = np.loadtxt(self.odom_file, skiprows=0)
        self.odoms_size = sum(1 for line in odom_txt)
        print("Loading Odoms")
        for i in tqdm(range(self.odoms_size)):
            odom = Odom(i, odom_txt[i][1:])
            self.odoms.append(odom)

    def loadLasers(self):
        laser_files = os.listdir(self.laser_folder)
        sort_nicely(laser_files)
        self.lasers_size = len(laser_files)
        print("Loading Lasers")
        for i in tqdm(range(self.lasers_size)):
            pcd_file_path = os.path.join(self.laser_folder, laser_files[i])
            laser = Laserscan(i, pcd_file_path, self.poses_time[i])
            self.lasers.append(laser)

    def loadImages(self):
        rgb_files = os.listdir(self.rgb_folder)
        depth_files = os.listdir(self.depth_folder)
        mask_files = os.listdir(self.mask_folder)
        sort_nicely(rgb_files)
        sort_nicely(depth_files)
        sort_nicely(mask_files)
        self.images_size = len(rgb_files)

        print("Loading Images")
        for i in tqdm(range(self.images_size)):
            rgb_file_path = os.path.join(self.rgb_folder, rgb_files[i])
            rgb = RGBImage(i, rgb_file_path, self.poses_time[i])
            self.images_rgbs.append(rgb)

            depth_file_path = os.path.join(self.depth_folder, depth_files[i])
            depth = DepthImage(i, depth_file_path, self.poses_time[i])
            self.images_depths.append(depth)

            mask_file_path = os.path.join(self.mask_folder, mask_files[i])
            mask = MaskImage(i, mask_file_path, self.poses_time[i])
            self.images_masks.append(mask)

sequences = sys.argv[1:]
print("Loading sequence(s):", sequences)
data = DataLoader(sequences[0])
print(data.poses[0].time,data.poses[0].x)
print(data.odoms[0].time,data.odoms[0].x)
print(data.imus[0].time,data.imus[0].ax)

print(data.images_depths[0].image)

cv2.imshow('img',data.images_rgbs[0].image)
cv2.waitKey(0)
cv2.imshow('mask rgb',data.images_masks[0].image_rgb)
cv2.waitKey(0)

            



    




