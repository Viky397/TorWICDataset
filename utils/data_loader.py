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

        if not os.path.exists(self.folder):
            raise RuntimeError("The folder " + self.folder + " does not exist")

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

        self.start_time = np.inf
        self.end_time = -np.inf

        self.loadPoses()
        self.loadOdoms()
        self.loadIMUs()
        self.loadLasers()
        self.loadImages()

        self.duration = self.end_time - self.start_time

    def loadPoses(self):
        # Load poses from the text file
        pose_txt = np.loadtxt(self.pose_file, skiprows=0)
        self.poses_size = sum(1 for line in pose_txt)
        print("Loading Poses")
        for i in tqdm(range(self.poses_size)):
            pose = Pose(i, pose_txt[i][1:])
            self.poses_time.append(pose.time)
            self.poses.append(pose)
        # Update trajectory start and finish time
        if self.poses[0].time < self.start_time:
            self.start_time =  self.poses[0].time
        if self.poses[-1].time > self.end_time:
            self.end_time =  self.poses[-1].time 

    def loadIMUs(self):
        # Load IMU meas. from the text file
        imu_txt = np.loadtxt(self.imu_file, skiprows=0)
        self.imus_size = sum(1 for line in imu_txt)
        print("Loading IMUs")
        for i in tqdm(range(self.imus_size)):
            imu = IMU(i, imu_txt[i][1:])
            self.imus.append(imu)
        # Update trajectory start and finish time
        if self.imus[0].time < self.start_time:
            self.start_time =  self.imus[0].time
        if self.imus[-1].time > self.end_time:
            self.end_time =  self.imus[-1].time 

    def loadOdoms(self):
        # Load odoms from the text file
        odom_txt = np.loadtxt(self.odom_file, skiprows=0)
        self.odoms_size = sum(1 for line in odom_txt)
        print("Loading Odoms")
        for i in tqdm(range(self.odoms_size)):
            odom = Odom(i, odom_txt[i][1:])
            self.odoms.append(odom)
        # Update trajectory start and finish time
        if self.odoms[0].time < self.start_time:
            self.start_time =  self.odoms[0].time
        if self.odoms[-1].time > self.end_time:
            self.end_time =  self.odoms[-1].time 

    def loadLasers(self):
        # Load Lidar meas. from the pcd files
        laser_files = os.listdir(self.laser_folder)
        sort_nicely(laser_files)
        self.lasers_size = len(laser_files)
        print("Loading Lasers")
        for i in tqdm(range(self.lasers_size)):
            pcd_file_path = os.path.join(self.laser_folder, laser_files[i])
            laser = Laserscan(i, pcd_file_path, self.poses_time[i])
            self.lasers.append(laser)
        # Update trajectory start and finish time
        if self.lasers[0].time < self.start_time:
            self.start_time =  self.lasers[0].time
        if self.lasers[-1].time > self.end_time:
            self.end_time =  self.lasers[-1].time 

    def loadImages(self):
        # Load RGB, depth and segmentations from the image files
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

    def applyTimeAndIndexOffset(self, dt, idx_pose, idx_imu, idx_odom, idx_laser, idx_image):
        # Apply a time offset to all data
        # Necessary to merge multiple trajectories
        for i in range(self.poses_size):
            self.poses[i].time -= dt
            self.poses[i].id += idx_pose
        for i in range(self.odoms_size):
            self.odoms[i].time -= dt
            self.odoms[i].id += idx_odom
        for i in range(self.imus_size):
            self.imus[i].time -= dt
            self.imus[i].id += idx_imu
        for i in range(self.lasers_size):
            self.lasers[i].time -= dt
            self.lasers[i].id += idx_laser
        for i in range(self.images_size):
            self.images_rgbs[i].time -= dt
            self.images_rgbs[i].id += idx_image
            self.images_depths[i].time -= dt
            self.images_depths[i].id += idx_image
            self.images_masks[i].time -= dt
            self.images_masks[i].id += idx_image
        self.start_time -= dt
        self.end_time -= dt

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        raise RuntimeError("Need at least one trajectory ID")
    sequences = sys.argv[1:]
    print("Loading sequence(s):", sequences)
    data = DataLoader(sequences[0])

    print(data.start_time, data.poses[0].time)
    print(data.end_time, data.poses[-1].time)

    print(data.poses[0].time,data.poses[0].x)
    print(data.odoms[0].time,data.odoms[0].x)
    print(data.imus[0].time,data.imus[0].ax)

    print(data.images_depths[0].image)

    cv2.imshow('img',data.images_rgbs[0].image)
    cv2.waitKey(0)
    cv2.imshow('mask bgr',data.images_masks[0].image_bgr)
    cv2.waitKey(0)

            



    




