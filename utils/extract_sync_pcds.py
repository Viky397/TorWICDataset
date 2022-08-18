import rosbag
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
from open3d_ros_helper import open3d_ros_helper as orh
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from multiprocessing.pool import ThreadPool

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

def dense_map(Pts, n, m, grid):
    ng = 2 * grid + 1
    
    mX = np.zeros((m,n)) + np.float("inf")
    mY = np.zeros((m,n)) + np.float("inf")
    mD = np.zeros((m,n))
    mX[np.int32(Pts[1]),np.int32(Pts[0])] = Pts[0] - np.round(Pts[0])
    mY[np.int32(Pts[1]),np.int32(Pts[0])] = Pts[1] - np.round(Pts[1])
    mD[np.int32(Pts[1]),np.int32(Pts[0])] = Pts[2]
    
    KmX = np.zeros((ng, ng, m - ng, n - ng))
    KmY = np.zeros((ng, ng, m - ng, n - ng))
    KmD = np.zeros((ng, ng, m - ng, n - ng))
    
    for i in range(ng):
        for j in range(ng):
            KmX[i,j] = mX[i : (m - ng + i), j : (n - ng + j)] - grid - 1 +i
            KmY[i,j] = mY[i : (m - ng + i), j : (n - ng + j)] - grid - 1 +i
            KmD[i,j] = mD[i : (m - ng + i), j : (n - ng + j)]
    S = np.zeros_like(KmD[0,0])
    Y = np.zeros_like(KmD[0,0])
    
    for i in range(ng):
        for j in range(ng):
            s = 1/np.sqrt(KmX[i,j] * KmX[i,j] + KmY[i,j] * KmY[i,j])
            Y = Y + s * KmD[i,j]
            S = S + s
    
    S[S == 0] = 1
    out = np.zeros((m,n))
    out[grid + 1 : -grid, grid + 1 : -grid] = Y/S
    return out

bag_in = rosbag.Bag('/home/jqian/Downloads/llr-0.bag', 'r')
#bag_in = rosbag.Bag('/home/jqian/Downloads/longHall1_forward.bag', 'r')


Width = 1280
Height = 720

fx = 608.5848999023438
cx = 642.230224609375
fy = 608.4966430664062
cy = 366.0254211425781

tf_p_x = 0.09842076  
tf_p_y = 0.17564679 
tf_p_z = -0.10376783
tf_r_x = -0.60163933  
tf_r_y = 0.40154873 
tf_r_z = -0.36939673  
tf_r_w = 0.5833822

tf_p = np.array([tf_p_x,tf_p_y,tf_p_z])
tf_r = np.array([tf_r_x,tf_r_y,tf_r_z,tf_r_w])
tf_C = R.from_quat(tf_r).as_matrix()
T = np.eye(4)
T[0:3,0:3] = tf_C
T[0:3,-1] = tf_p
T_inv = np.linalg.inv(T)

P = np.array([[fx, 0, cx, 0],[0, fy, cy, 0],[0, 0, 1, 0],[0, 0, 0, 1]])

K = np.array([[608.584900,0.000000,642.230225],[0.000000,608.496643,366.025421],[0.000000,0.000000,1.000000]])
D = np.array([0.121450,-2.315923,0.000223,-0.000383,1.495202,0.011486,-2.141217,1.416827])

print(T)
print(P)


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
print("Num pcds: ", len(pcd_msg))
####################################################################################
print("Sync..")
filt_pcd = []
filt_pcd_time = []
filt_pcd_dt = []

for t in tqdm(rgb_time):
    idx = pcd_time.index(min(pcd_time, key=lambda x:abs(x - t)))
    filt_pcd.append(pcd_msg[idx])
    filt_pcd_time.append(pcd_time[idx])
    filt_pcd_dt.append(abs(pcd_time[idx] - t))

        
print("Num filt_pcd: ", len(filt_pcd))
print("Avg dt: ", np.mean(np.array(filt_pcd_dt)))

print("Saving...")

def do_work(i):
#for i in tqdm(range(len(filt_pcd))):
    print('Working on', i)
    np_arr = np.fromstring(rgb_msg[i].data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_np = cv2.undistort(image_np, K, D)
    o3dpc = orh.rospc_to_o3dpc(filt_pcd[i])

    #o3dpc = passthrough(o3dpc,[1,5,-3,3,-1,5])
    #o3d.io.write_point_cloud("/home/jqian/Downloads/" + str(i).zfill(4) + "_before.ply", o3dpc)

    pts_orig = np.asarray(o3dpc.points)
    pts_orig = pts_orig[~np.isnan(pts_orig).any(axis=1)]
    pts_orig_h = np.hstack((pts_orig, np.ones((pts_orig.shape[0], 1))))
    pts_h = np.matmul(pts_orig_h, T_inv.transpose())
    pts_h /= pts_h[:, 2:3]
    pts_h[:, 2] = 1

    pts_img = np.matmul(pts_h, P.transpose())
    
    mask = np.where((pts_img[:, 0] >= 0) &
                    (pts_img[:, 0] <= Width - 1) &
                    (pts_img[:, 1] >= 0) &
                    (pts_img[:, 1] <= Height - 1) &
                    (1.0/pts_img[:, 3] > 0))
    pts_img = pts_img[mask]
    depths = 1.0/pts_img[:, 3]
    colors = depths / 8.0

    img_depth = np.ones((Height,Width), dtype=np.uint16) * 65535
    h_len = 3
    v_len = 5
    for j in range(pts_img.shape[0]):
        cp = (round(pts_img[j,0]),round(pts_img[j,1]))
        #tlp = (max(round(pts_img[j,0])-side_len,0),max(round(pts_img[j,1])-side_len,0))
        #brp = (min(round(pts_img[j,0])+side_len,Width),min(round(pts_img[j,1])+side_len,Height))
        
        
        #cv2.circle(image_np, cp, 1,(100+128*min(colors[j],1),200-128*min(colors[j],1),255*min(colors[j],1)))
        
        for k in range(round(max(round(pts_img[j,0])-h_len,0)), round(min(round(pts_img[j,0])+h_len,Width))):
            for l in range(round(max(round(pts_img[j,1])-v_len,0)), round(min(round(pts_img[j,1])+v_len,Height))):

                img_depth[l, k] = min(round(depths[j] * 1000), img_depth[l, k])
        #cv2.rectangle(img_depth,tlp,brp,min(round(depths[j] * 1000), 65535),cv2.FILLED)
    
    #dm2 = dense_map([pts_img[:,0],pts_img[:,1],depths],Width, Height, 5)
    
    #cv2.imshow('rgb', image_np)
    #cv2.waitKey(0)
    #print(img_depth)
    #cv2.imshow('dep', img_depth)
    #cv2.waitKey(0)
    cv2.imwrite("/home/jqian/Downloads/demo-longloopreverse/image_0/" + str(i).zfill(6) + ".png", image_np)
    #cv2.imwrite("/home/jqian/Downloads/demo-longloopreverse/depth/" + str(i).zfill(6) + ".png", img_depth)
    print('Done', i)
    
with ThreadPool(processes=10) as pool:
    #pool.map(do_work, [i for i in range(len(filt_pcd))])
    pool.map(do_work, [i for i in range(11)])

'''
pose_msg_x = []
pose_msg_y = []
pose_msg_z = []

pose_msg_qx = []
pose_msg_qy = []
pose_msg_qz = []
pose_msg_qw = []

sec = []
nsec = []
id_val = []

count = 0
if not os.path.exists(folder + "LaserScans"):
    os.makedirs(folder + "LaserScans")

for i in range(len(depth_cam_info)):
    
    filt_pose_msg[i].header.stamp = depth_cam_info_time[i]
    filt_depth_msg[i].header.stamp = depth_cam_info_time[i]
    filt_rgb_cam_info[i].header.stamp = depth_cam_info_time[i]
    filt_rgb_msg[i].header.stamp = depth_cam_info_time[i]

    for msg in filt_tf[i].transforms:
        msg.header.stamp = depth_cam_info_time[i]
    
    for msg in filt_static_tf[i].transforms:
        msg.header.stamp = depth_cam_info_time[i]

    cvb=CvBridge()
    depth_mem=None
    rgb_mem=None
    first_flag=True

    extract_depth(filt_depth_msg[i], count)
    extract_rgb(filt_rgb_msg[i], count)

    id_val.append(count)
    sec.append(depth_cam_info_time[i].secs)
    nsec.append(depth_cam_info_time[i].nsecs)
    pose_msg_x.append(filt_pose_msg[i].pose.pose.position.x)
    pose_msg_y.append(filt_pose_msg[i].pose.pose.position.y)
    pose_msg_z.append(filt_pose_msg[i].pose.pose.position.z)
    pose_msg_qx.append(filt_pose_msg[i].pose.pose.orientation.x)
    pose_msg_qy.append(filt_pose_msg[i].pose.pose.orientation.y)
    pose_msg_qz.append(filt_pose_msg[i].pose.pose.orientation.z)
    pose_msg_qw.append(filt_pose_msg[i].pose.pose.orientation.w)

    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(filt_laser_msg[i])
    pc = pypcd.PointCloud.from_msg(pc2_msg)
    pc.save(folder + "/LaserScans/" + str(count).zfill(4) + ".pcd")

    count +=1 
'''
