import rosbag
import numpy as np
import os
import sys
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


#target = 'Hallway_Straight_CW'
#bag_name_1 = 'hallway_straight_cw_part_1.bag'

target = sys.argv[1]
bag_name_1 = sys.argv[2]
bag_name_2 = sys.argv[3]

bag_in_1 = rosbag.Bag('/home/jqian/Downloads/TORWIC/JUN15/'+bag_name_1, 'r')
bag_in_2 = rosbag.Bag('/home/jqian/Downloads/TORWIC/JUN15/'+bag_name_2, 'r')
bags = [bag_in_1, bag_in_2]

rgb_left_out      = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/image_left/'
rgb_right_out     = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/image_right/'
depth_left_out    = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/depth_left/'
depth_right_out   = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/depth_right/'
lidar_out         = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/lidar/'
imu_left_out      = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/imu_left.txt'
imu_right_out     = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/imu_right.txt'
time_out          = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/frame_times.txt'


Width = 1280
Height = 720


rgb_left_msg = []
rgb_left_time = []

rgb_right_msg = []
rgb_right_time = []

depth_left_msg = []
depth_left_time = []

depth_right_msg = []
depth_right_time = []

imu_left_msg = []
imu_left_time = []

imu_right_msg = []
imu_right_time = []

pcd_msg = []
pcd_time = []

print("Extracting Topics...")
for bag in bags:
    for topic, msg, t in bag:

        if topic == "/left_azure/rgb/image_raw/compressed":
            nsec = msg.header.stamp.to_sec()
            rgb_left_time.append(nsec)
            rgb_left_msg.append(msg)

        if topic == "/right_azure/rgb/image_raw/compressed":
            nsec = msg.header.stamp.to_sec()
            rgb_right_time.append(nsec)
            rgb_right_msg.append(msg)

        if topic == "/left_azure/depth_to_rgb/image_raw":
            nsec = msg.header.stamp.to_sec()
            depth_left_time.append(nsec)
            depth_left_msg.append(msg)

        if topic == "/right_azure/depth_to_rgb/image_raw":
            nsec = msg.header.stamp.to_sec()
            depth_right_time.append(nsec)
            depth_right_msg.append(msg)

        if topic == "/left_azure/imu":
            nsec = msg.header.stamp.to_sec()
            imu_left_time.append(nsec)
            imu_left_msg.append(msg)

        if topic == "/right_azure/imu":
            nsec = msg.header.stamp.to_sec()
            imu_right_time.append(nsec)
            imu_right_msg.append(msg)
        
        if topic == "/os_cloud_node/points":
            nsec = msg.header.stamp.to_sec()
            pcd_time.append(nsec)
            pcd_msg.append(msg)
    bag.close()
    del bag
    gc.collect()

print("Done Topic Extraction")
print("Num images: ", len(rgb_left_msg),len(rgb_right_msg))
print("Num depths: ", len(depth_left_msg),len(depth_right_msg))
print("Num IMUs: ", len(imu_left_msg), len(imu_right_msg))
print("Num pcds: ", len(pcd_msg))

init_time = min(pcd_time[0], imu_left_time[0], imu_right_time[0])
####################################################################################
print("Sync..")
filt_rgb_left = []
filt_rgb_left_time = []
filt_rgb_left_dt = []

filt_rgb_right = []
filt_rgb_right_time = []
filt_rgb_right_dt = []

filt_depth_left = []
filt_depth_left_time = []
filt_depth_left_dt = []

filt_depth_right = []
filt_depth_right_time = []
filt_depth_right_dt = []

for t in tqdm(pcd_time):
    rgb_left_idx = rgb_left_time.index(min(rgb_left_time, key=lambda x:abs(x - t)))
    rgb_right_idx = rgb_right_time.index(min(rgb_right_time, key=lambda x:abs(x - t)))
    depth_left_idx = depth_left_time.index(min(depth_left_time, key=lambda x:abs(x - t)))
    depth_right_idx = depth_right_time.index(min(depth_right_time, key=lambda x:abs(x - t)))

    filt_rgb_left.append(rgb_left_msg[rgb_left_idx])
    filt_rgb_right.append(rgb_right_msg[rgb_right_idx])
    filt_depth_left.append(depth_left_msg[depth_left_idx])
    filt_depth_right.append(depth_right_msg[depth_right_idx])


    filt_rgb_left_time.append(rgb_left_time[rgb_left_idx])
    filt_rgb_right_time.append(rgb_right_time[rgb_right_idx])
    filt_depth_left_time.append(depth_left_time[depth_left_idx])
    filt_depth_right_time.append(depth_right_time[depth_right_idx])

    filt_rgb_left_dt.append(abs(rgb_left_time[rgb_left_idx] - t))
    filt_rgb_right_dt.append(abs(rgb_right_time[rgb_right_idx] - t))
    filt_depth_left_dt.append(abs(depth_left_time[depth_left_idx] - t))
    filt_depth_right_dt.append(abs(depth_right_time[depth_right_idx] - t))

del rgb_left_msg
del rgb_right_msg
del depth_left_msg
del depth_right_msg
gc.collect()

print("Num filt_rgb: ", len(filt_rgb_left), len(filt_rgb_right))
print("Avg dt: ", np.mean(np.array(filt_rgb_left_dt)), np.mean(np.array(filt_rgb_right_dt)))
print("Num filt_depth: ", len(filt_depth_left), len(filt_depth_right))
print("Avg dt: ", np.mean(np.array(filt_depth_left_dt)), np.mean(np.array(filt_depth_right_dt)))

print("Saving...")

def do_work(i):
#for i in tqdm(range(len(filt_pcd))):
    print('Working on', i)
    file_name = str(i).zfill(6)
    
    np_arr = np.frombuffer(filt_rgb_left[i].data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(rgb_left_out + file_name + ".png", image_np)

    np_arr = np.frombuffer(filt_rgb_right[i].data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(rgb_right_out + file_name + ".png", image_np)

    cvb=CvBridge()
    cv_image = cvb.imgmsg_to_cv2(filt_depth_left[i],filt_depth_left[i].encoding)
    image_np= np.array(cv_image,dtype=np.uint16)
    cv2.imwrite(depth_left_out + file_name + ".png", image_np)

    cv_image = cvb.imgmsg_to_cv2(filt_depth_right[i],filt_depth_right[i].encoding)
    image_np= np.array(cv_image,dtype=np.uint16)
    cv2.imwrite(depth_right_out + file_name + ".png", image_np)

    pcd_ros = pcd_msg[i]
    pcd_o3d = orh.rospc_to_o3dpc(pcd_ros) 
    o3d.io.write_point_cloud(lidar_out + file_name + ".pcd", pcd_o3d)

    print('Done', i)

def write_imu_message_to_file(init_time, msg, time, file):
    N = len(msg)
    np_arr = np.zeros((N,7))
    for i in range(N):
        np_arr[i,0] = time[i] - init_time
        np_arr[i,1] = msg[i].angular_velocity.x
        np_arr[i,2] = msg[i].angular_velocity.y
        np_arr[i,3] = msg[i].angular_velocity.z
        np_arr[i,4] = msg[i].linear_acceleration.x
        np_arr[i,5] = msg[i].linear_acceleration.y
        np_arr[i,6] = msg[i].linear_acceleration.z

    np.savetxt(file, np_arr)

time_np = np.array(pcd_time).reshape((-1,1)) - init_time
np.savetxt(time_out,time_np)

write_imu_message_to_file(init_time, imu_left_msg, imu_left_time, imu_left_out)
write_imu_message_to_file(init_time, imu_right_msg, imu_right_time, imu_right_out)

#context = get_context('forkserver')
#with ProcessPoolExecutor(1, mp_context=context) as executor:
with ThreadPool(processes=1) as executor:
    executor.map(do_work, [i for i in range(len(pcd_time))])


