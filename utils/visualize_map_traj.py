import sys
from calendar import c
from turtle import pos
import open3d as o3d
import numpy as np
import copy
from pypcd import pypcd
import os
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

def passthrough(pcd, bounds):
	min_x = bounds[0]
	max_x = bounds[1]
	min_y = bounds[2]
	max_y = bounds[3]
	min_z = bounds[4]
	max_z = bounds[5]
	
	min_bound = np.array([min_x,min_y,min_z])
	max_bound = np.array([max_x,max_y,max_z])
	
	bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound,max_bound)
	return pcd.crop(bbox)

def draw(map, traj):
    map_temp = copy.deepcopy(map)
    traj_temp = copy.deepcopy(traj)
    map_temp.paint_uniform_color([1, 0, 0])
    traj_temp.paint_uniform_color([0, 1, 0])

    map_temp = passthrough(map,[-100,100,-100,100,-0.5,3])
    o3d.visualization.draw_geometries([map_temp, traj_temp])


if __name__ == "__main__":
    target = sys.argv[1]
    print("Visualize: ", target)

    #target = 'Hallway_Straight_CW'
    path = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/'
    poses = np.loadtxt(path + "traj_gt.txt")
    map_ap1 = o3d.io.read_point_cloud("/home/jqian/ChangingWarehouseDataset/utils/groundtruth_map.ply")
    
    T_rob_full = []
    traj_rob = []

    idxes = []

    for i in tqdm(range(poses.shape[0])):
        
        idxes.append(int(poses[i,0]))
        
        p = poses[i,1:4] # xyz
        q = poses[i,4:8] #quat

        c = R.from_quat(q).as_matrix()

        T_rob = np.eye(4)
        T_rob[0:3,0:3] = c
        T_rob[0:3,-1] = p

        T_rob_full.append(T_rob)
        traj_rob.append(p)
        
        
    pcd_rob = o3d.geometry.PointCloud()
    pcd_rob.points = o3d.utility.Vector3dVector(traj_rob)

    draw(map_ap1, pcd_rob)