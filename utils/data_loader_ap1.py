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
    def __init__(self, folder):
        self.folder = folder
        self.rgb_folder = self.folder + "/image/"
        self.depth_folder = self.folder + "/depth/"
        self.mask_folder = self.folder + "/segmentation_greyscale/"
        self.pose_file = self.folder + "/evaluation/traj_gt.txt"
        self.time_file = self.folder + "/times.txt"

        if not os.path.exists(self.folder):
            raise RuntimeError("The folder " + self.folder + " does not exist")

        self.images_size = 0
        self.images_rgbs = []
        self.images_depths = []
        self.images_masks = []

        self.poses_size = 0
        self.poses_time = []
        self.poses = []

        self.times = np.loadtxt(self.time_file)
        self.N = len(self.times) - 1

        self.start_time = self.times[0]
        self.end_time = self.times[self.N-1]

        

        self.loadPoses()
        self.loadImages()

        self.duration = self.end_time - self.start_time

    def loadPoses(self):
        # Load poses from the text file
        pose_txt = np.loadtxt(self.pose_file, skiprows=0)
        self.poses_size = self.N
        print("Loading Poses")
        for i in tqdm(range(self.N)):
            pose = Pose(i, self.times[i], pose_txt[i][0:])
            self.poses_time.append(pose.time)
            self.poses.append(pose)

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
        for i in tqdm(range(self.N)):
            rgb_file_path = os.path.join(self.rgb_folder, rgb_files[i])
            rgb = RGBImage(i, rgb_file_path, self.poses_time[i])
            self.images_rgbs.append(rgb)

            depth_file_path = os.path.join(self.depth_folder, depth_files[i])
            depth = DepthImage(i, depth_file_path, self.poses_time[i])
            self.images_depths.append(depth)

            mask_file_path = os.path.join(self.mask_folder, mask_files[i])
            mask = MaskImage(i, mask_file_path, self.poses_time[i])
            self.images_masks.append(mask)

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

    print(data.images_depths[0].getImage())

    data.images_rgbs[0].imshow()
    data.images_masks[0].imshow()
    data.images_depths[0].imshow()


            



    




