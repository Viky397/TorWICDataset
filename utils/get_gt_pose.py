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

from multiprocessing import get_context
from concurrent.futures import ProcessPoolExecutor

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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    target_temp = passthrough(target_temp,[-100,100,-100,100,-100,100])
    source_temp = passthrough(source_temp,[-100,100,-100,100,-100,100])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def draw_registration_result_more(source, target, odom):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    odom_temp = copy.deepcopy(odom)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 1, 0])
    odom_temp.paint_uniform_color([0, 0, 1])
    # target_temp = passthrough(target_temp,[-100,100,-100,100,1.5,2])
    source_temp = passthrough(source_temp,[-100,100,-100,100,1.5,2])
    o3d.visualization.draw_geometries([source_temp, target_temp, odom_temp])

###### STEP 2: transform ORB poses with extrinsic

target = sys.argv[1]

#target = 'Hallway_Straight_CW'
path = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/'
file_path = path+'lidar/'
target = o3d.io.read_point_cloud("/home/jqian/ChangingWarehouseDataset/utils/groundtruth_map.ply")
target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=20))
#o3d.visualization.draw_geometries([target])

times = np.loadtxt(path+'frame_times.txt')
scan_files = sorted(os.listdir(file_path))
T_final = []
N = len(scan_files)


print("Number of frames:", N)
    
    
def do_work(i):	
	print("Start:", i)

	scan = scan_files[i]
	if (i==0):
		t_init = np.eye(4)
	else:
		t_init = T_final[-1]

	source_ouster = o3d.io.read_point_cloud(file_path + scan)

	####### STEP 3: for each ouster lidar scan, transform with corresponding {T_init} 

	pts_orig = np.asarray(source_ouster.points)
	pts_orig = pts_orig[~np.isnan(pts_orig).any(axis=1)]
	pts_orig_h = np.hstack((pts_orig, np.ones((pts_orig.shape[0], 1))))
	pts_h = np.matmul(pts_orig_h, t_init.transpose())
	pts_final = pts_h[:,:3]

	pcd_ouster = o3d.geometry.PointCloud()
	pcd_ouster.points = o3d.utility.Vector3dVector(pts_final)
	# o3d.visualization.draw_geometries([pcd_ouster])


	#### STEP 4: run ICP to align each ouster lidar to AP1 to get {T_corr}         
	
	if (i==0):
		threshold = 5.0
	else:
		threshold = 1.0
	
	trans_init = np.eye(4)
	
	pcd_ouster_crop = passthrough(pcd_ouster,[-100,100,-100,100,2,100])
	target_crop = passthrough(target,[-100,100,-100,100,2,100])
	
	#draw_registration_result(pcd_ouster_crop, target_crop, trans_init)
	# print("Apply point-to-plane ICP")

	reg_p2pl = o3d.pipelines.registration.registration_icp(
		pcd_ouster_crop, target_crop, threshold, trans_init,
		o3d.pipelines.registration.TransformationEstimationPointToPlane())

	# print("Transformation is:")
	# print(reg_p2pl.transformation)

	#draw_registration_result(pcd_ouster_crop, target_crop, reg_p2pl.transformation)

	T_rough = reg_p2pl.transformation
	pcd_ouster_crop.transform(T_rough) 

	threshold = 0.3
	reg_p2pl = o3d.pipelines.registration.registration_icp(
		pcd_ouster_crop, target_crop, threshold, trans_init,
		o3d.pipelines.registration.TransformationEstimationPointToPlane())

	T_fine = reg_p2pl.transformation
	
	
	#draw_registration_result(pcd_ouster_crop, target_crop, T_fine)
	
	T_complete = T_fine @ T_rough
	#print(T_complete)

	final_pose = T_complete @ t_init #@ np.linalg.inv(T_complete)
	#print(final_pose)

	z = final_pose[2,3]
	if z>0.25:
		for j in reversed(range(len(T_final))):
			temp_z = T_final[j][2,3]
			if temp_z<0.25:
				final_pose[2,3] = temp_z
				break

	T_final.append(final_pose)
	
	pose_converted_p = final_pose[0:3,3]
	pose_converted_q = R.from_matrix(final_pose[0:3,0:3]).as_quat()

	pose_full = np.hstack((times[i], pose_converted_p, pose_converted_q)).reshape(1, 8)

	print("Done:", i)

	return pose_full

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    res = (cumsum[N:] - cumsum[:-N]) / float(N)
    return np.concatenate((x[0:N-1], res)) 
    
if __name__ == "__main__":
    final_poses_full = []
    context = get_context('forkserver')
    
    idxes = [i for i in range(len(scan_files))]
    #idxes = [i for i in range(10)]

    with ProcessPoolExecutor(1, mp_context=context) as executor:
        for result in executor.map(do_work, idxes):
            final_poses_full.append(result)
    final_poses_full = sorted(final_poses_full, key=lambda x: x[0,0])
    final_poses_full = np.array(final_poses_full).squeeze()
    print(final_poses_full)
    final_poses_full[:,3] = running_mean(final_poses_full[:,3], 5)
    print(final_poses_full)
    


    for i in tqdm(range(len(final_poses_full))):
        with open(path + "traj_gt.txt", "a") as f:
            np.savetxt(f, [final_poses_full[i,:].tolist()], fmt=["%s", "%s", "%s", "%s", "%s", "%s", "%s", "%s"])
            
	
            
