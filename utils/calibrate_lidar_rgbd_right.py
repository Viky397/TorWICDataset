import rosbag
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
import copy
from tqdm import tqdm
from open3d_ros_helper import open3d_ros_helper as orh
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from multiprocessing.pool import ThreadPool
from multiprocessing import get_context
from concurrent.futures import ProcessPoolExecutor

import tf.transformations as tr
from avg_quat import averageQuaternions

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


def draw_pcds(a, b):
    a_temp = copy.deepcopy(a)
    b_temp = copy.deepcopy(b)
    a_temp.paint_uniform_color([1, 0, 0])
    b_temp.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([a_temp, b_temp])

def unproject(img, K, width, height):
    pts = []
    fx = K[0,0]
    cx = K[0,2]
    fy = K[1,1]
    cy = K[1,2]
    for u in range(0, width):
        for v in range(0, height):
            z = img[v,u]
            if z != 0 and z < 65534:
                d = z / 1000.0
                x = (u-cx)*d/fx;
                y = (v-cy)*d/fy;
                pt = [x,y,d]
                pts.append(pt)
    pts = np.array(pts)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts[:, :3])

    return pc


bag_in = rosbag.Bag('/home/jqian/Downloads/POV-SLAM/Experiments/demo-longloopreverse/long_loop_reverse.bag', 'r')
#bag_in = rosbag.Bag('/home/jqian/Downloads/cam_lidar_cal/left.bag', 'r')

Width = 1280
Height = 720

# init guess
T_init =np.array([[-0.4159633, -0.0612660,  0.9073153, 0.12361888],
[-0.9092719,  0.0125366, -0.4160137, -0.05236871],
[0.0141129, -0.9980428, -0.0609223, -0.11045261],
[0	,0	,0	,1]])

T_init =np.array([[-0.3952075,  -0.09230251,  0.91394271,  0.07686256],
 [-0.91807052,  0.00617267, -0.39636905, -0.15441064],
 [ 0.03094439 ,-0.99571188, -0.0871797,  -0.1026438 ],
 [ 0.    ,      0.     ,     0.       ,   1.        ]])


# factory
#P = np.array([[fx, 0, cx, 0],[0, fy, cy, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
#K = np.array([[fx,0.,cx],[0.,fy,cy],[0.,0.,1.]])
#D = np.array([0.121450,-2.315923,0.000223,-0.000383,1.495202,0.011486,-2.141217,1.416827])


# matlab
D = np.array([0.0873365707229730, -0.0669194159864173, 0.00495094616995581, 0.000778746904798700, 0.0221870772417529])
K = np.array([613.100554293744, 0.0, 638.086832160361, 0.0, 613.903691840614, 378.314715743038, 0.0, 0.0, 1.0]).reshape((3,3))
P = np.array([613.100554293744, 0.0, 638.086832160361, 0.0, 0.0, 613.903691840614, 378.314715743038, 0.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 0, 1]).reshape((4,4))

print(T_init)
print(P)
print(K)

depth_msg = []
depth_time = []

pcd_msg = []
pcd_time = []

for topic, msg, t in bag_in:

    #if topic == "/left_azure/depth_to_rgb/image_raw/compressed":
    if topic == "/right_azure/depth_to_rgb/image_raw":
        nsec = msg.header.stamp.to_sec()
        depth_time.append(nsec)
        depth_msg.append(msg)
    
    if topic == "/os_cloud_node/points":
        nsec = msg.header.stamp.to_sec()
        pcd_time.append(nsec)
        pcd_msg.append(msg)

    if len(pcd_msg) > 50:
        break
    

print("Done Topic Extraction")
print("Num images: ", len(depth_msg))
print("Num pcds: ", len(pcd_msg))
####################################################################################
print("Sync..")
filt_img = []
filt_img_time = []
filt_img_dt = []

for t in tqdm(pcd_time):
    idx = depth_time.index(min(depth_time, key=lambda x:abs(x - t)))
    dt = abs(depth_time[idx] - t)
    if (dt<0.5):
        filt_img.append(depth_msg[idx])
        filt_img_time.append(depth_time[idx])
        filt_img_dt.append(dt)

        
print("Num filt_img: ", len(filt_img))
print("Avg dt: ", np.mean(np.array(filt_img_dt)))

depth_msg.clear()

Knew = K.copy()
unmap1, unmap2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), Knew, (Width, Height), cv2.CV_32F)
unmap1, unmap2 = cv2.convertMaps(unmap1, unmap2, cv2.CV_16SC2)

p_all = np.empty((0,3),float)
q_all = np.empty((0,4),float)

indices = [i for i in range(len(filt_img_time)) if i % 10 == 0]
for i in indices:
    print('Working on', i)
    #####
    #np_arr = np.fromstring(filt_img[i].data, np.uint8)
    #image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    bridge = CvBridge()
    image_np = bridge.imgmsg_to_cv2(filt_img[i], desired_encoding='passthrough')
    #####

    image_rect = cv2.remap(image_np, unmap1, unmap2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT) 

    rgbdpc = unproject(image_np, K, Width, Height)
    rgbdpc.transform(T_init)

    o3dpc = orh.rospc_to_o3dpc(pcd_msg[i])
    o3dpc = passthrough(o3dpc,[0.2,100,-100,100,-5,5])
    o3dpc.remove_non_finite_points()
    o3dpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    threshold = 0.5

    trans_init = np.eye(4)

    #print("Initial alignment")
    
    evaluation = o3d.pipelines.registration.evaluate_registration(rgbdpc, o3dpc,
                                                        threshold, trans_init)

    #print("Apply point-to-point ICP")
    reg = o3d.pipelines.registration.registration_icp(
        rgbdpc, o3dpc, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())


    T_rough = reg.transformation

    rgbdpc.transform(T_rough)
    
    #print("Fine alignment")
    threshold = 0.05
    evaluation = o3d.pipelines.registration.evaluate_registration(rgbdpc, o3dpc,
                                                        threshold, trans_init)                                     

    #print("Apply point-to-point ICP")
    reg = o3d.pipelines.registration.registration_icp(
        rgbdpc, o3dpc, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 2000))

    #print("Transformation is:")
    T_fine = reg.transformation
    #print(T_fine)

    rgbdpc.transform(T_fine)
    
    T_final = T_fine @ T_rough @ T_init


    p_final = T_final[0:3,3]
    q_final = tr.quaternion_from_matrix(T_final)

    print("p_final: ", p_final)
    print("q_final: ", q_final)

    p_all = np.vstack([p_all,p_final])
    q_all = np.vstack([q_all,q_final])
        

    draw_pcds(rgbdpc, o3dpc)

p_avg = np.mean(p_all, axis=0)
q_avg = averageQuaternions(q_all)
    
print("p_avg: ", p_avg)
print("q_avg: ", q_avg)

c_ret = tr.quaternion_matrix(q_avg)[0:3,0:3]

T_ret = np.eye(4)
T_ret[0:3,0:3] = c_ret
T_ret[0:3,-1] = p_avg

print(T_ret)