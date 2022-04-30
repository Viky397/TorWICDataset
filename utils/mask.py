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
        # Image file
        self.image_path = file

    def getImageIdx(self):
        return cv2.imread(self.image_path, -1)

    def getImageRGB(self):
        return mask_colors[self.getImageIdx()]

    def getImageBGR(self):
        return cv2.cvtColor(self.getImageRGB(), cv2.COLOR_RGB2BGR)
    
    def imshow(self):
        cv2.imshow('Color Image', self.getImageBGR())
        cv2.waitKey(0)

    def toROSMsg(self):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.getImageRGB(), 'rgb8')
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        return msg

    def getCameraInfo(self):
        return getCameraInfoMsg(self.id, self.time, self.frame)