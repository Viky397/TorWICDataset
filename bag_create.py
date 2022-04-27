import rosbag
import os
from cv_bridge import CvBridge
import cv2
import re
from re import I
import numpy as np
from tqdm import tqdm
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import rospy
import pypcd

folder = "Test1/Test1-1/"
bag_name = "1-1.bag"

bag_write = rosbag.Bag("/home/veronica/Downloads/Tests/" + folder + bag_name, 'w')
########## Add Poses #################################################################

pose_sec = []
pose_nsec = []
x = []
y = []
z = []
qx = []
qy = []
qz = []
qw = []

array_info = np.loadtxt("/home/veronica/Downloads/Tests/" + folder + "poses.txt", skiprows=0)

print("Append Pose info")
num_lines = sum(1 for line in array_info)

for i in tqdm(range(num_lines)):

    pose_sec.append(array_info[i][1])
    pose_nsec.append(array_info[i][2])
    x.append(array_info[i][3])
    y.append(array_info[i][4])
    z.append(array_info[i][5])
    qx.append(array_info[i][6])
    qy.append(array_info[i][7])
    qz.append(array_info[i][8])
    qw.append(array_info[i][9])

def to_pose_stamped(frame_id, child_frame_id, i):
    pose_msg = Odometry()
    pose_msg.header.seq = i
    pose_msg.header.stamp = rospy.Time.from_sec(float(pose_sec[i]) + float(pose_nsec[i])/1e9)
    pose_msg.header.frame_id = frame_id
    pose_msg.child_frame_id = child_frame_id 
    pose_msg.pose.pose.position.x = x[i]
    pose_msg.pose.pose.position.y = y[i]
    pose_msg.pose.pose.position.z = z[i]
    pose_msg.pose.pose.orientation.x = qx[i]
    pose_msg.pose.pose.orientation.y = qy[i]
    pose_msg.pose.pose.orientation.z = qz[i]
    pose_msg.pose.pose.orientation.w = qw[i]
    return pose_msg

for i in tqdm(range(len(x))):
    pose_message = to_pose_stamped("map", "base_link", i)
    bag_write.write('/slam/pose', pose_message, pose_message.header.stamp)

print("Append TF info")
for i in tqdm(range(num_lines)):
    tf_msgs = TFMessage()
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = i
    tf_stamped.header.stamp = rospy.Time.from_sec(float(pose_sec[i]) + float(pose_nsec[i])/1e9)
    tf_stamped.header.frame_id = "map"
    tf_stamped.child_frame_id = "base_link"
    tf_stamped.transform.translation.x = x[i]
    tf_stamped.transform.translation.y = y[i]
    tf_stamped.transform.translation.z = z[i]
    tf_stamped.transform.rotation.x = qx[i]
    tf_stamped.transform.rotation.y = qy[i]
    tf_stamped.transform.rotation.z = qz[i]
    tf_stamped.transform.rotation.w = qw[i]

    tf_msgs.transforms.append(tf_stamped)
    bag_write.write('/tf', tf_msgs, rospy.Time.from_sec(float(pose_sec[i]) + float(pose_nsec[i])/1e9))

print("Append camera-TF info")
tf_msgs = TFMessage()
tf_stamped = TransformStamped()
tf_stamped.header.seq = 0
tf_stamped.header.stamp = rospy.Time.from_sec(float(pose_sec[0]) + float(pose_nsec[0])/1e9)
tf_stamped.header.frame_id = "base_link"
tf_stamped.child_frame_id = "realsense_front_camera_color_optical_frame"
tf_stamped.transform.translation.x = 0.520
tf_stamped.transform.translation.y = 0.032
tf_stamped.transform.translation.z = 0.011
tf_stamped.transform.rotation.x = -0.431
tf_stamped.transform.rotation.y = 0.429
tf_stamped.transform.rotation.z = -0.562
tf_stamped.transform.rotation.w = 0.561
tf_msgs.transforms.append(tf_stamped)

print("Append IMU-TF info")
tf_stamped = TransformStamped()
tf_stamped.header.seq = 0
tf_stamped.header.stamp = rospy.Time.from_sec(float(pose_sec[0]) + float(pose_nsec[0])/1e9)
tf_stamped.header.frame_id = "base_link"
tf_stamped.child_frame_id = "imu_link"
tf_stamped.transform.translation.x = 0.192
tf_stamped.transform.translation.y = -0.086
tf_stamped.transform.translation.z = 0.185
tf_stamped.transform.rotation.x = 0.000
tf_stamped.transform.rotation.y = 0.000
tf_stamped.transform.rotation.z = -0.383
tf_stamped.transform.rotation.w = 0.924
tf_msgs.transforms.append(tf_stamped)

print("Append LiDAR-TF info")
tf_stamped = TransformStamped()
tf_stamped.header.seq = 0
tf_stamped.header.stamp = rospy.Time.from_sec(float(pose_sec[0]) + float(pose_nsec[0])/1e9)
tf_stamped.header.frame_id = "base_link"
tf_stamped.child_frame_id = "front_laser"
tf_stamped.transform.translation.x = 0.456
tf_stamped.transform.translation.y = 0.000
tf_stamped.transform.translation.z = 0.172
tf_stamped.transform.rotation.x = 0.000
tf_stamped.transform.rotation.y = 0.000
tf_stamped.transform.rotation.z = 0.000
tf_stamped.transform.rotation.w = 1.0

tf_msgs.transforms.append(tf_stamped)
bag_write.write('/tf_static', tf_msgs, rospy.Time.from_sec(float(pose_sec[0]) + float(pose_nsec[0])/1e9))

########## Add ODOM #################################################################

sec = []
nsec = []
x = []
y = []
z = []
qx = []
qy = []
qz = []
qw = []

twist_x = []
twist_y = []
twist_z = []
twist_qx = []
twist_qy = []
twist_qz = []

array_info = np.loadtxt("/home/veronica/Downloads/Tests/" + folder + "odom.txt", skiprows=0)

print("Append Odometry info")
num_lines = sum(1 for line in array_info)
for i in tqdm(range(num_lines)):

    sec.append(array_info[i][1])
    nsec.append(array_info[i][2])
    x.append(array_info[i][3])
    y.append(array_info[i][4])
    z.append(array_info[i][5])
    qx.append(array_info[i][6])
    qy.append(array_info[i][7])
    qz.append(array_info[i][8])
    qw.append(array_info[i][9])
    twist_x.append(array_info[i][10])
    twist_y.append(array_info[1][11])
    twist_z.append(array_info[i][12])
    twist_qx.append(array_info[i][13])
    twist_qy.append(array_info[i][14])
    twist_qz.append(array_info[i][15])

def to_odom_stamped(frame_id, child_frame_id, i):
    odom_msg = Odometry()
    odom_msg.header.seq = i
    odom_msg.header.stamp = rospy.Time.from_sec(float(sec[i]) + float(nsec[i])/1e9)
    odom_msg.header.frame_id = frame_id
    odom_msg.child_frame_id = child_frame_id 
    odom_msg.pose.pose.position.x = x[i]
    odom_msg.pose.pose.position.y = y[i]
    odom_msg.pose.pose.position.z = z[i]
    odom_msg.pose.pose.orientation.x = qx[i]
    odom_msg.pose.pose.orientation.y = qy[i]
    odom_msg.pose.pose.orientation.z = qz[i]
    odom_msg.pose.pose.orientation.w = qw[i]
    odom_msg.twist.twist.linear.x = twist_x[i]
    odom_msg.twist.twist.linear.y = twist_y[i]
    odom_msg.twist.twist.linear.z = twist_z[i]
    odom_msg.twist.twist.angular.x = twist_qx[i]
    odom_msg.twist.twist.angular.y = twist_qy[i]
    odom_msg.twist.twist.angular.z = twist_qz[i]
    return odom_msg

for i in tqdm(range(len(x))):
    odom_message = to_odom_stamped("map", "odom_link", i)
    bag_write.write('/velocity_control/odom', odom_message, odom_message.header.stamp)

########## Add IMU #################################################################

sec = []
nsec = []
x = []
y = []
z = []
qx = []
qy = []
qz = []

array_info = np.loadtxt("/home/veronica/Downloads/Tests/" + folder + "imu.txt", skiprows=0)

print("Append IMU info")
num_lines = sum(1 for line in array_info)
for i in tqdm(range(num_lines)):

    sec.append(array_info[i][1])
    nsec.append(array_info[i][2])
    x.append(array_info[i][3])
    y.append(array_info[i][4])
    z.append(array_info[i][5])
    qx.append(array_info[i][6])
    qy.append(array_info[i][7])
    qz.append(array_info[i][8])

def to_imu_stamped(frame_id, i):
    imu_msg = Imu()
    imu_msg.header.seq = i
    imu_msg.header.stamp = rospy.Time.from_sec(float(sec[i]) + float(nsec[i])/1e9)
    imu_msg.header.frame_id = frame_id
    imu_msg.linear_acceleration.x = x[i]
    imu_msg.linear_acceleration.y = y[i]
    imu_msg.linear_acceleration.z = z[i]
    imu_msg.angular_velocity.x = qx[i]
    imu_msg.angular_velocity.y = qy[i]
    imu_msg.angular_velocity.z = qz[i]
    return imu_msg

for i in tqdm(range(len(x))):
    imu_message = to_imu_stamped("imu_link", i)
    bag_write.write('/imu/data', imu_message, imu_message.header.stamp)

########## Add LiDAR #################################################################

def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    l.sort(key=alphanum_key)

lidar_path = "/home/veronica/Downloads/Tests/" + folder + "LaserScans/"

print("Append LiDAR info")

def to_laser_stamped(frame_id, pcd_path, count):
    pc = pypcd.PointCloud.from_path(pcd_path)
    pc_data = pc.pc_data

    laser_msg = LaserScan()
    laser_msg.header.seq = count
    laser_msg.header.stamp = rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9)
    laser_msg.header.frame_id = frame_id
    laser_msg.angle_min = -2.35619449615
    laser_msg.angle_max = 2.35619449615
    laser_msg.angle_increment = 0.00436332309619
    laser_msg.time_increment = 2.08333385672e-05
    laser_msg.scan_time = 0.0299999993294
    laser_msg.range_min = 0.0
    laser_msg.range_max = 40.0
    ranges = []
    for row in pc_data:
        ranges.append(np.sqrt(row["x"]**2 + row["y"]**2))
    laser_msg.ranges = ranges
    laser_msg.intensities = pc_data["intensity"]
    
    return laser_msg

list_masks = os.listdir(lidar_path)
sort_nicely(list_masks)

count = 0
for pcd in tqdm(list_masks):
    pcd_path = os.path.join(lidar_path, pcd)
    laser_message = to_laser_stamped("front_laser", pcd_path, count)
    bag_write.write('/front/scan', laser_message, laser_message.header.stamp)
    count +=1

########################### ADD IMAGES ###############################################3
rgb_path = "/home/veronica/Downloads/Tests/" + folder + "RGB/"
depth_path = "/home/veronica/Downloads/Tests/" + folder + "Depth/"
mask_path = "/home/veronica/Downloads/Tests/" + folder + "Segmentation/"

# RGB
list_masks = os.listdir(rgb_path)
sort_nicely(list_masks)

count = 0
for img in list_masks:
    cv_image = cv2.imread(os.path.join(rgb_path, img), -1)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, 'passthrough')
    image_message.header.seq = count
    image_message.header.stamp = rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9)
    image_message.header.frame_id = 'realsense_front_camera_color_optical_frame'
    bag_write.write('/front/realsense/color/image_raw', image_message, rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9))

    count +=1

# Depth
list_masks = os.listdir(depth_path)
sort_nicely(list_masks)

count = 0
for img in list_masks:
    cv_image = cv2.imread(os.path.join(depth_path, img), -1)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, 'passthrough')
    image_message.header.seq = count
    image_message.header.stamp = rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9)
    image_message.header.frame_id = 'realsense_front_camera_color_optical_frame'
    bag_write.write('/front/realsense/aligned_depth_to_color/image_raw', image_message, rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9))

    count +=1

# Segmentation

colours = np.array([[0,0,0], #background #black
[255,255,255], #ceiling # white
[0,191,255], #ego_vehicle #baby blue
[0,255,0], #wall_fence_pillar #green
[255,0,102], #misc_static_feature #hot pink
[153,0,204], #rack_shelf #dark purple
[51, 51, 204], #goods_material #dark blue
[0, 153, 153], #fixed_machinery #blue_green
[255, 204, 255], #cart_pallet_jack #baby pink
[255, 153, 0], #pylon_cone #orange
[255, 255, 0], #text_region #yellow
[255, 0, 0], #misc_non_static_feature #red
[204, 102, 255], #person #baby purple
[255, 77, 77], #fork_truck #watermelon  
[0, 153, 51], #misc_dynamic_feature #dark green
[191, 191, 191]], dtype=np.uint8) #driveable_ground #grey

list_masks = os.listdir(mask_path)
sort_nicely(list_masks)

count = 0
for img in list_masks:
    cv_image1 = cv2.imread(os.path.join(mask_path, img), -1)
    cv_img = colours[cv_image1]
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_img, 'rgb8')
    image_message.header.seq = count
    image_message.header.stamp = rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9)
    image_message.header.frame_id = 'realsense_front_camera_color_optical_frame'
    bag_write.write('/front/realsense/segmentation/image_raw', image_message, rospy.Time.from_sec(float(pose_sec[count]) + float(pose_nsec[count])/1e9))

    count +=1

bag_write.close()
