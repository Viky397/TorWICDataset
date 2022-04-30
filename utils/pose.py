import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from time_util import toSec

# Wrapper for pose data
class Pose:
    def __init__(self, id, data = [0 for i in range(9)]):
        assert(len(data) == 9)

        # Frame
        self.frame = "map"
        self.child_frame = "base_link"
        # Topic
        self.topic = "/slam/pose"
        # ID
        self.id = id
        # Timestamp
        self.sec = int(data[0])
        self.nsec = int(data[1])
        self.time = toSec(self.sec, self.nsec)

        # Position
        self.x = float(data[2])
        self.y = float(data[3])
        self.z = float(data[4])

        # Orientation
        self.qx = float(data[5])
        self.qy = float(data[6])
        self.qz = float(data[7])
        self.qw = float(data[8])

    def toROSMsg(self):
        msg = Odometry()
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.x = self.qx
        msg.pose.pose.orientation.y = self.qy
        msg.pose.pose.orientation.z = self.qz
        msg.pose.pose.orientation.w = self.qw
        return msg

    def toROSTF(self):
        tf_msg = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header.seq = self.id
        tf_stamped.header.stamp = rospy.Time.from_sec(self.time)
        tf_stamped.header.frame_id = self.frame
        tf_stamped.child_frame_id = self.child_frame
        tf_stamped.transform.translation.x = self.x
        tf_stamped.transform.translation.y = self.y
        tf_stamped.transform.translation.z = self.z
        tf_stamped.transform.rotation.x = self.qx
        tf_stamped.transform.rotation.y = self.qy
        tf_stamped.transform.rotation.z = self.qz
        tf_stamped.transform.rotation.w = self.qw
        tf_msg.transforms.append(tf_stamped)
        return tf_msg