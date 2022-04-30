import cv2
import rospy
from cv_bridge import CvBridge
from time_util import toSec
from calibration import getCameraInfoMsg

# Wrapper for RGB image
class RGBImage:
    def __init__(self, id, file, time):
        # Frame
        self.frame = "realsense_front_camera_color_optical_frame"
        # Topic
        self.topic = "/front/realsense/color/"
        # ID
        self.id = id
        # Timestamp
        self.time = time
        # Image file
        self.image_path = file

    def getImageBGR(self):
        return cv2.imread(self.image_path, -1)

    def getImageRGB(self):
        return cv2.cvtColor(self.getImageBGR(), cv2.COLOR_BGR2RGB)
    
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