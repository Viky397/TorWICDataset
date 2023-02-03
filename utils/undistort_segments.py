from mask_colors import mask_colors

import numpy as np
import os
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
from sort_nicely import sort_nicely

from scipy.spatial.transform import Rotation as R
from multiprocessing.pool import ThreadPool


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


folder_mono_in = '/home/jqian/Downloads/demo-longloopreverse/segmentation_greyscale_1'
#folder_out = '/home/jqian/Downloads/demo-longloopreverse/segmentation_colour_0'
folder_rect_out = '/home/jqian/Downloads/demo-longloopreverse/segmentation_colour_undistort_0'
folder_mono_rect_out = '/home/jqian/Downloads/demo-longloopreverse/segmentation_greyscale_0'


masks_mono_in = os.listdir(folder_mono_in)
sort_nicely(masks_mono_in)

####################################################################################
Knew = K.copy()
unmap1, unmap2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), Knew, (Width, Height), cv2.CV_32F)
unmap1, unmap2 = cv2.convertMaps(unmap1, unmap2, cv2.CV_16SC2)

print("Saving...")

def do_work(i):
#for i in tqdm(range(len(filt_pcd))):
    print('Working on', i)
    mask_mono_path = os.path.join(folder_mono_in, masks_mono_in[i])
    out_rect_path = os.path.join(folder_rect_out, masks_mono_in[i])
    out_mono_rect_path = os.path.join(folder_mono_rect_out, masks_mono_in[i])
    #out_path = os.path.join(folder_out, masks_mono_in[i])

    image_mono = cv2.imread(mask_mono_path, -1)
    image_rgb = np.zeros((Height,Width,3), dtype=np.uint8)
    #for i in range(Width):
    #    for j in range(Height):
    #        image_rgb[j,i,:] = np.flip(mask_colors[image_mono[j,i]])

    #image_rect = cv2.remap(image_rgb, unmap1, unmap2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    image_mono_rect = cv2.remap(image_mono, unmap1, unmap2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT) 
   
    #cv2.imwrite(out_rect_path, image_rect)
    cv2.imwrite(out_mono_rect_path, image_mono_rect)
    print('Done', i)
    
with ThreadPool(processes=10) as pool:
    pool.map(do_work, [i for i in range(len(masks_mono_in))])
    #pool.map(do_work, [i for i in range(11)])
    #print('hi')
