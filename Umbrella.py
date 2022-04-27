import rosbag
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import pypcd
from sensor_msgs.msg import PointCloud2

bag_in = rosbag.Bag('/home/veronica/Downloads/Tests/Test1/Test1-1/test1-1_orig.bag', 'r')   
folder = "Test1/Test1-1/"

pose_time = []
pose_msg = []

rgb_msg = []
rgb_time = []

rgb_cam_info = []
rgb_cam_info_time = []

tf_msg = []
tf_time = []

tf_static_msg = []
tf_static_time = []

depth_msg = []
depth_time = []

imu_msg = []

odom_msg = []

laser_msg = []
laser_time = []

for topic, msg, t in bag_in:

    if topic == "/slam/pose":
        nsec = msg.header.stamp.to_nsec()
        pose_time.append(nsec)
        pose_msg.append(msg)

    if topic == "/front/scan":
        nsec = msg.header.stamp.to_nsec()
        laser_time.append(nsec)
        laser_msg.append(msg)

    if topic == "/front/realsense/aligned_depth_to_color/image_raw":
        nsec = msg.header.stamp.to_nsec()
        depth_time.append(nsec)
        depth_msg.append(msg)
    
    if topic == "/front/realsense/color/camera_info":
        nsec = msg.header.stamp.to_nsec()
        rgb_cam_info_time.append(nsec)
        rgb_cam_info.append(msg)
    
    if topic == "/front/realsense/color/image_raw":
        nsec = msg.header.stamp.to_nsec()
        rgb_time.append(nsec)
        rgb_msg.append(msg)
    
    if topic == "/tf":
        nsec = msg.transforms[0].header.stamp.to_nsec()
        tf_time.append(nsec)
        tf_msg.append(msg)
    
    if topic == "/tf_static":
        nsec = msg.transforms[0].header.stamp.to_nsec()
        tf_static_time.append(nsec)
        tf_static_msg.append(msg)

    if topic == "/imu/data":
        nsec = msg.header.stamp.to_nsec()
        imu_msg.append(msg)

    if topic == "/velocity_control/odom":
        nsec = msg.header.stamp.to_nsec()
        odom_msg.append(msg)

print("Done Topic Extraction")

############### DEPTH ################################################################
print("Extracting Depth")

def extract_depth(msg, count):
    global video
    global depth_mem
    global first_flag
    
    cv_image = cvb.imgmsg_to_cv2(msg,msg.encoding)

    numpy_image= np.array(cv_image,dtype=np.uint16)

    if not os.path.exists(folder + "Depth"):
        os.makedirs(folder + "Depth")
        
    if first_flag == True:
        depth_mem = np.copy(numpy_image)

        img = cv2.imwrite(folder + "Depth/" + str(count).zfill(4) + ".png", numpy_image)
        
        first_flag=False
    if (depth_mem==numpy_image).all() :
        return
        
    else:
        depth_mem = np.copy(numpy_image)
        img = cv2.imwrite(folder + "Depth/" + str(count).zfill(4) + ".png", numpy_image)
	
############### RGB ################################################################
print("Extracting RGB")

def extract_rgb(msg, count):
    global video
    global rgb_mem
    global first_flag
    
    cv_image = cvb.imgmsg_to_cv2(msg,"bgr8")
    
    image_normal= np.array(cv_image)

    if not os.path.exists(folder + "RGB"):
        os.makedirs(folder + "RGB")

    if first_flag == True :
        rgb_mem = np.copy(image_normal)
        img = cv2.imwrite(folder + "RGB/" + str(count).zfill(4) + ".png", image_normal)
	
        first_flag=False

    elif np.array_equal(rgb_mem,image_normal) :
        return

    else:
        rgb_mem = np.copy(image_normal)
        img = cv2.imwrite(folder + "/RGB/" + str(count).zfill(4) + ".png", image_normal)    

####################################################################################

depth_cam_info = []
depth_cam_info_time = []
filt_pose_msg = []
filt_laser_msg = []
filt_depth_msg = []
filt_rgb_msg = []
filt_rgb_cam_info = []
filt_tf = []
filt_static_tf = []

list_msgs = ["/front/realsense/aligned_depth_to_color/camera_info"]

count = 0
for topic, msg, t in bag_in:
    if topic in list_msgs:

        nsec = msg.header.stamp.to_nsec()

        filt_pose_msg.append(pose_msg[pose_time.index(min(pose_time, key=lambda x:abs(x - nsec)))])
        
        filt_laser_msg.append(laser_msg[laser_time.index(min(laser_time, key=lambda x:abs(x - nsec)))])

        filt_depth_msg.append(depth_msg[depth_time.index(min(depth_time, key=lambda x:abs(x - nsec)))])
        filt_rgb_msg.append(rgb_msg[rgb_time.index(min(rgb_time, key=lambda x:abs(x - nsec)))])
        filt_rgb_cam_info.append(rgb_cam_info[rgb_cam_info_time.index(min(rgb_cam_info_time, key=lambda x:abs(x - nsec)))])
        filt_tf.append(tf_msg[tf_time.index(min(tf_time, key=lambda x:abs(x - nsec)))])
        filt_static_tf.append(tf_static_msg[tf_static_time.index(min(tf_static_time, key=lambda x:abs(x - nsec)))])

        depth_cam_info.append(msg)
        depth_cam_info_time.append(msg.header.stamp)
        count+=1

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

    rospy.init_node("extractingImages")
    rate=rospy.Rate(1)
    rospy.loginfo("Extracting Depth Images")
    extract_depth(filt_depth_msg[i], count)

    rospy.loginfo("Extracting RGB Images")
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
data = np.column_stack([id_val, sec, nsec, pose_msg_x, pose_msg_y, pose_msg_z, pose_msg_qx, pose_msg_qy, pose_msg_qz, pose_msg_qw])
np.savetxt(folder + "/poses.txt", data, fmt=['%d', '%d', '%d','%f','%f','%f','%f','%f','%f','%f'])

sec_imu = []
nsec_imu = []
id_val_imu = []

imu_msg_x = []
imu_msg_y = []
imu_msg_z = []

imu_msg_qx = []
imu_msg_qy = []
imu_msg_qz = []

count_imu = 0
for i in range(len(imu_msg)):

    id_val_imu.append(count_imu)
    sec_imu.append(imu_msg[i].header.stamp.secs)
    nsec_imu.append(imu_msg[i].header.stamp.nsecs)
    imu_msg_x.append(imu_msg[i].linear_acceleration.x)
    imu_msg_y.append(imu_msg[i].linear_acceleration.y)
    imu_msg_z.append(imu_msg[i].linear_acceleration.z)
    imu_msg_qx.append(imu_msg[i].angular_velocity.x)
    imu_msg_qy.append(imu_msg[i].angular_velocity.y)
    imu_msg_qz.append(imu_msg[i].angular_velocity.z)

    count_imu +=1

data_imu = np.column_stack([id_val_imu, sec_imu, nsec_imu, imu_msg_x, imu_msg_y, imu_msg_z, imu_msg_qx, imu_msg_qy, imu_msg_qz])
np.savetxt(folder + "/imu.txt", data_imu, fmt=['%d', '%d', '%d','%f','%f','%f','%f','%f','%f'])


sec_odom = []
nsec_odom = []
id_val_odom = []

odom_msg_x = []
odom_msg_y = []
odom_msg_z = []

odom_msg_qx = []
odom_msg_qy = []
odom_msg_qz = []
odom_msg_qw = []

odom_twist_x = []
odom_twist_y = []
odom_twist_z = []

odom_twist_qx = []
odom_twist_qy = []
odom_twist_qz = []

count_odom = 0
for i in range(len(odom_msg)):

    id_val_odom.append(count_odom)
    sec_odom.append(odom_msg[i].header.stamp.secs)
    nsec_odom.append(odom_msg[i].header.stamp.nsecs)
    odom_msg_x.append(odom_msg[i].pose.pose.position.x)
    odom_msg_y.append(odom_msg[i].pose.pose.position.y)
    odom_msg_z.append(odom_msg[i].pose.pose.position.z)
    odom_msg_qx.append(odom_msg[i].pose.pose.orientation.x)
    odom_msg_qy.append(odom_msg[i].pose.pose.orientation.y)
    odom_msg_qz.append(odom_msg[i].pose.pose.orientation.z)
    odom_msg_qw.append(odom_msg[i].pose.pose.orientation.w)

    odom_twist_x.append(odom_msg[i].twist.twist.linear.x)
    odom_twist_y.append(odom_msg[i].twist.twist.linear.y)
    odom_twist_z.append(odom_msg[i].twist.twist.linear.z)
    odom_twist_qx.append(odom_msg[i].twist.twist.angular.x)
    odom_twist_qy.append(odom_msg[i].twist.twist.angular.y)
    odom_twist_qz.append(odom_msg[i].twist.twist.angular.z)

    count_odom +=1

data_odom = np.column_stack([id_val_odom, sec_odom, nsec_odom, odom_msg_x, odom_msg_y, odom_msg_z, odom_msg_qx, odom_msg_qy, odom_msg_qz, odom_msg_qw,
                            odom_twist_x, odom_twist_y, odom_twist_z, odom_twist_qx, odom_twist_qy, odom_twist_qz])
np.savetxt(folder + "/odom.txt", data_odom, fmt=['%d', '%d', '%d','%f','%f','%f','%f','%f','%f','%f', '%f','%f','%f','%f','%f','%f'])
