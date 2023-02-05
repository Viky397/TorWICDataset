import os
import numpy as np
import shutil

from sort_nicely import sort_nicely

path_dataset1 = "/home/jqian/Downloads/AP1_Test_Route/output/racking"
path_dataset2 = "/home/jqian/Downloads/AP1_Test_Route/output/aisle"

path_dest = "/home/jqian/Downloads/AP1_Test_Route/output/fused_rack_aisle"

rpath_image = "/image/"
rpath_depth = "/depth/"
rpath_segmentation = "/segmentation_greyscale/"

file_calib = "/cam_extrinsics.txt"
file_time = "/times.txt"
file_gt_traj = "/evaluation/traj_gt.txt"

def move_file(rpath):
    data1_files = os.listdir(path_dataset1 + rpath)
    sort_nicely(data1_files)

    data2_files = os.listdir(path_dataset2 + rpath)
    sort_nicely(data2_files)

    data1_file_ids = np.array([int(img.split('.')[0]) for img in data1_files])
    data1_last_file_id = data1_file_ids[-1]

    data2_file_ids = np.array([int(img.split('.')[0]) for img in data2_files])
    data2_dest_file_ids = data2_file_ids + data1_last_file_id + 1

    for i in range(len(data1_file_ids)):
        idx = data1_file_ids[i]
        src_name = str(idx).zfill(6) + ".png"
        path_src = path_dataset1 + rpath + src_name
        dst_name = str(idx).zfill(6) + ".png"
        path_dst = path_dest + rpath + dst_name
        shutil.copy2(path_src, path_dst)

    for i in range(len(data2_file_ids)):
        idx_src = data2_file_ids[i]
        idx_dst = data2_dest_file_ids[i]
        src_name = str(idx_src).zfill(6) + ".png"
        path_src = path_dataset2 + rpath + src_name
        dst_name = str(idx_dst).zfill(6) + ".png"
        path_dst = path_dest + rpath + dst_name
        shutil.copy2(path_src, path_dst)

move_file(rpath_image)
move_file(rpath_depth)
move_file(rpath_segmentation)

data1_calib = np.loadtxt(path_dataset1 + file_calib)
data2_calib = np.loadtxt(path_dataset2 + file_calib)

dest_calib = np.vstack((data1_calib,data2_calib))
np.savetxt(path_dest+file_calib, dest_calib, fmt=["%s", "%s", "%s", "%s", "%s", "%s", "%s"])

data1_time = np.loadtxt(path_dataset1 + file_time)
data2_time = np.loadtxt(path_dataset2 + file_time)
data2_time += data1_time[-1] + (data1_time[-1] - data1_time[-2])

dest_time = np.hstack((data1_time,data2_time)).reshape((-1,1))
np.savetxt(path_dest+file_time, dest_time, fmt=["%s"])

data1_traj_gt = np.loadtxt(path_dataset1 + file_gt_traj)[:,1:]
data2_traj_gt = np.loadtxt(path_dataset2 + file_gt_traj)[:,1:]

dest_traj_gt = np.vstack((data1_traj_gt,data2_traj_gt))
dest_traj_gt = np.hstack((dest_time,dest_traj_gt))
np.savetxt(path_dest+file_gt_traj, dest_traj_gt, fmt=["%s", "%s", "%s", "%s", "%s", "%s", "%s", "%s"])

