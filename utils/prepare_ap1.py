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

bag_in = rosbag.Bag('/home/jqian/Downloads/demo-longloopreverse/long_loop_reverse.bag', 'r')
seg_mono_in = '/home/jqian/Downloads/demo-longloopreverse/segmentation_greyscale_1'

rgb_out      = '/home/jqian/Downloads/demo-longloopreverse/image/'
depth_out    = '/home/jqian/Downloads/demo-longloopreverse/depth/'
seg_mono_out = '/home/jqian/Downloads/demo-longloopreverse/segmentation_greyscale/'
seg_rgb_out  = '/home/jqian/Downloads/demo-longloopreverse/segmentation_colour/'
time_out  = '/home/jqian/Downloads/demo-longloopreverse/times.txt'
ext_out  = '/home/jqian/Downloads/demo-longloopreverse/cam_extrinsics.txt'

masks_mono_in = os.listdir(seg_mono_in)
sort_nicely(masks_mono_in)

Width = 1280
Height = 720

tf_p = np.array([ 0.12944592,  0.04299934, -0.11374339])
tf_r = np.array( [-0.61167248,  0.39292797, -0.3567415,   0.58668551])

tf_C = R.from_quat(tf_r).as_matrix()
T = np.eye(4)
T[0:3,0:3] = tf_C
T[0:3,-1] = tf_p

T_inv = np.linalg.inv(T)

# factory
'''
fx = 608.5848999023438
cx = 642.230224609375
fy = 608.4966430664062
cy = 366.0254211425781
P = np.array([[fx, 0, cx, 0],[0, fy, cy, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
K = np.array([[fx,0.,cx],[0.,fy,cy],[0.,0.,1.]])
D = np.array([0.121450,-2.315923,0.000223,-0.000383,1.495202,0.011486,-2.141217,1.416827])
'''

# matlab
D = np.array([0.0754678811883405, -0.0314893098110791, 0.00101924098521082, 0.00209529701722382, -0.000935223936623030])
K = np.array([621.397474650905, 0.0, 649.644481364277, 0.0, 620.649424677401, 367.908146431575, 0.0, 0.0, 1.0]).reshape((3,3))
P = np.array([621.397474650905, 0.0, 649.644481364277, 0.0, 0.0, 620.649424677401, 367.908146431575, 0.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 0, 1]).reshape((4,4))

Knew = K.copy()
unmap1, unmap2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), Knew, (Width, Height), cv2.CV_32F)
unmap1, unmap2 = cv2.convertMaps(unmap1, unmap2, cv2.CV_16SC2)

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

filt_seg_idx = []

for t in tqdm(pcd_time):
    idx = rgb_time.index(min(rgb_time, key=lambda x:abs(x - t)))
    filt_rgb.append(rgb_msg[idx])
    filt_rgb_time.append(rgb_time[idx])
    filt_rgb_dt.append(abs(rgb_time[idx] - t))

# workaround
for t in tqdm(filt_rgb_time):
    idx = rgb_time.index(min(rgb_time, key=lambda x:abs(x - t)))
    filt_seg_idx.append(idx)

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
    image_np = cv2.undistort(image_np, K, D)

    o3dpc = orh.rospc_to_o3dpc(pcd_msg[i])

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
        #cp = (round(pts_img[j,0]),round(pts_img[j,1]))
        #cv2.circle(image_np, cp, 2,(100+128*min(colors[j],1),200-128*min(colors[j],1),255*min(colors[j],1)))
        
        
        for k in range(round(max(round(pts_img[j,0])-h_len,0)), round(min(round(pts_img[j,0])+h_len,Width))):
            for l in range(round(max(round(pts_img[j,1])-v_len,0)), round(min(round(pts_img[j,1])+v_len,Height))):
                img_depth[l, k] = min(round(depths[j] * 1000), img_depth[l, k])
        
    
    cv2.imwrite(rgb_out + file_name + ".png", image_np)
    cv2.imwrite(depth_out + file_name + ".png", img_depth)

    mask_file_name = str(filt_seg_idx[i]).zfill(6)
    mask_mono_path = os.path.join(seg_mono_in, mask_file_name+".png")
    mask_mono = cv2.imread(mask_mono_path, -1)
    mask_mono_rect = cv2.remap(mask_mono, unmap1, unmap2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)

    mask_rgb = np.zeros((Height,Width,3), dtype=np.uint8)
    for u in range(Width):
        for v in range(Height):
            mask_rgb[v,u,:] = np.flip(mask_colors[mask_mono_rect[v,u]])

    cv2.imwrite(seg_rgb_out+file_name+".png", mask_rgb)
    cv2.imwrite(seg_mono_out+file_name+".png", mask_mono_rect)

    print('Done', i)

time_np = np.array(pcd_time).reshape((-1,1)) - pcd_time[0]
np.savetxt(time_out,time_np)

def gen_extrinsics():
	ext_val = np.concatenate([tf_p, tf_r]).reshape((1,7))
	ext_np = np.repeat(ext_val, repeats=len(pcd_time), axis=0)
	np.savetxt(ext_out,ext_np)
	
gen_extrinsics()

  
#with ThreadPool(processes=1) as pool:
#    pool.map(do_work, [i for i in range(len(filt_rgb))])


