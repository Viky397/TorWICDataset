import rosbag
import numpy as np
import os
import gc
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
from open3d_ros_helper import open3d_ros_helper as orh
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from multiprocessing.pool import ThreadPool
from multiprocessing import get_context
from concurrent.futures import ProcessPoolExecutor
from sort_nicely import sort_nicely
from mask_colors import mask_colors


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

bag_in = rosbag.Bag('/home/jqian/Downloads/AP1_Test_Route/aisle1_reverse.bag', 'r')

rgb_out   = '/home/jqian/Downloads/AP1_Test_Route/output/aisle/image_raw/'
lidar_out   = '/home/jqian/Downloads/AP1_Test_Route/output/aisle/lidar/'
time_out  = '/home/jqian/Downloads/AP1_Test_Route/output/aisle/times.txt'


Width = 1280
Height = 720


rgb_msg = []
rgb_time = []

pcd_msg = []
pcd_time = []

for topic, msg, t in bag_in:

    if topic == "/left_azure/rgb/image_raw/compressed":
        nsec = msg.header.stamp.to_sec()
        rgb_time.append(nsec)
        rgb_msg.append(msg)
    
    if topic == "/os_cloud_node/points":
        nsec = msg.header.stamp.to_sec()
        pcd_time.append(nsec)
        pcd_msg.append(msg)

print("Done Topic Extraction")
print("Num images: ", len(rgb_msg))
print("Num pcds: ", len(pcd_msg), len(pcd_time))
####################################################################################
print("Sync..")
filt_rgb = []
filt_rgb_time = []
filt_rgb_dt = []

for t in tqdm(pcd_time):
    idx = rgb_time.index(min(rgb_time, key=lambda x:abs(x - t)))
    filt_rgb.append(rgb_msg[idx])
    filt_rgb_time.append(rgb_time[idx])
    filt_rgb_dt.append(abs(rgb_time[idx] - t))


del bag_in
del rgb_msg
gc.collect()

print("Num filt_rgb: ", len(filt_rgb))
print("Avg dt: ", np.mean(np.array(filt_rgb_dt)))

print("Saving...")

def do_work(i):
#for i in tqdm(range(len(filt_pcd))):
    print('Working on', i)
    file_name = str(i).zfill(6)
    
    np_arr = np.fromstring(filt_rgb[i].data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    #cv2.imwrite(rgb_out + file_name + ".png", image_np)

    pcd_ros = pcd_msg[i]
    pcd_o3d = orh.rospc_to_o3dpc(pcd_ros) 
    o3d.io.write_point_cloud(lidar_out + file_name + ".pcd", pcd_o3d)

    print('Done', i)

#time_np = np.array(pcd_time).reshape((-1,1)) - pcd_time[0]
#np.savetxt(time_out,time_np)

  
with ThreadPool(processes=1) as pool:
    pool.map(do_work, [i for i in range(len(filt_rgb))])


