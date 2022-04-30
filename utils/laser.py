import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
# Use this commit for python3: https://github.com/dimatura/pypcd/pull/35
from pypcd import pypcd
from time_util import toSec

# Wrapper for lidar data
class Laserscan:
    def __init__(self, id, file, time):
        # Frame
        self.frame = "front_laser"
        # Topic
        self.topic = "/front/scan"
        # ID
        self.id = id
        # Timestamp
        self.time = time
        # Pointcloud data
        self.pcd = pypcd.PointCloud.from_path(file)

    def toROSMsg(self):
        pc_data = self.pcd.pc_data
        msg = LaserScan()
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        msg.angle_min = -2.35619449615
        msg.angle_max = 2.35619449615
        msg.angle_increment = 0.00436332309619
        msg.time_increment = 2.08333385672e-05
        msg.scan_time = 0.0299999993294
        msg.range_min = 0.0
        msg.range_max = 40.0
        msg.ranges = [np.sqrt(row["x"]**2 + row["y"]**2) for row in pc_data]
        msg.intensities = pc_data["intensity"]
        return msg