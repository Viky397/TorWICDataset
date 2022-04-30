import cv2
import rospy
from cv_bridge import CvBridge
from time_util import toSec
from calibration import getCameraInfoMsg

# Wrapper for depth image
class DepthImage:
    def __init__(self, id, file, time):
        # Frame
        self.frame = "realsense_front_camera_color_optical_frame"
        # Topic
        self.topic = "/front/realsense/aligned_depth_to_color/"
        # ID
        self.id = id
        # Timestamp
        self.time = time
        # Pointcloud data
        self.image = cv2.imread(file, -1)

    def toROSMsg(self):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.image, 'passthrough')
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        return msg

    def getCameraInfo(self):
        return getCameraInfoMsg(self.id, self.time, self.frame)