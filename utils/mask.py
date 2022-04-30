import cv2
import rospy
from cv_bridge import CvBridge
from mask_colors import mask_colors
from time_util import toSec
from calibration import getCameraInfoMsg

# Wrapper for segmentation image
class MaskImage:
    def __init__(self, id, file, time):
        # Frame
        self.frame = "realsense_front_camera_color_optical_frame"
        # Topic
        self.topic = "/front/realsense/segmentation/"
        # ID
        self.id = id
        # Timestamp
        self.time = time
        # Segmentation image
        self.image_idx = cv2.imread(file, -1)
        self.image_rgb = mask_colors[self.image_idx]
        # For visualization with cv2.imshow
        self.image_bgr = cv2.cvtColor(self.image_rgb, cv2.COLOR_RGB2BGR)

    def toROSMsg(self):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.image_rgb, 'rgb8')
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        return msg

    def getCameraInfo(self):
        return getCameraInfoMsg(self.id, self.time, self.frame)