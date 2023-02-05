import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf.transformations as tr

from time_util import toSec
import numpy as np
from scipy.spatial.transform import Rotation as R


# Wrapper for pose data
class Pose:
    def __init__(self, id, time, data = [0 for i in range(9)]):
        print(data)
        assert(len(data) == 8)

        # Frame
        self.frame = "map"
        self.child_frame = "base_link"
        # Topic
        self.topic = "/slam/pose"
        # ID
        self.id = id
        # Timestamp
        self.time = time

        # Position
        self.x = float(data[1])
        self.y = float(data[2])
        self.z = float(data[3])

        # Orientation
        self.qx = float(data[4])
        self.qy = float(data[5])
        self.qz = float(data[6])
        self.qw = float(data[7])

        self.p = np.array([self.x,self.y,self.z])
        self.q = np.array([self.qx,self.qy,self.qz,self.qw])

        c = R.from_quat(self.q).as_matrix()
        T_mc = np.eye(4)
        T_mc[0:3,0:3] = c
        T_mc[0:3,-1] = self.p

        p_lc = np.array([0.12944592,  0.04299934, -0.11374339])
        q_lc = np.array([-0.61167248,  0.39292797, -0.3567415,   0.58668551])
        c_lc = R.from_quat(q_lc).as_matrix()
        T_lc = np.eye(4)
        T_lc[0:3,0:3] = c_lc
        T_lc[0:3,-1] = p_lc

        self.T_ml  = T_mc #T_lc @ T_mc @ np.linalg.inv(T_lc)

        p_rob = self.T_ml[0:3,3]
        q_rob = tr.quaternion_from_matrix(self.T_ml)

        self.x = p_rob[0]
        self.y = p_rob[1]
        self.z = p_rob[2]

        # Orientation
        self.qx = q_rob[0]
        self.qy = q_rob[1]
        self.qz = q_rob[2]
        self.qw = q_rob[3]

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