import rospy
from nav_msgs.msg import Odometry
from time_util import toSec

# Wrapper for odomtery data
class Odom:
    def __init__(self, id, data = [0 for i in range(15)]):
        assert(len(data) == 15)

        # Frame
        self.frame = "map"
        self.child_frame = "base_link"
        # Topic
        self.topic = "/velocity_control/odom"
        # ID
        self.id = id
        # Timestamp
        self.sec = int(data[0])
        self.nsec = int(data[1])
        self.time = toSec(self.sec, self.nsec)
        
        # Accumulated odom
        # Position
        self.x = float(data[2])
        self.y = float(data[3])
        self.z = float(data[4])
        # Orientation
        self.qx = float(data[5])
        self.qy = float(data[6])
        self.qz = float(data[7])
        self.qw = float(data[8])

        # Twist
        # Linear velocity
        self.vx = float(data[9])
        self.vy = float(data[10])
        self.vz = float(data[11])
        # Angular velocity
        self.wx = float(data[12])
        self.wy = float(data[13])
        self.wz = float(data[14])

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
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.linear.z = self.vz
        msg.twist.twist.angular.x = self.wx
        msg.twist.twist.angular.y = self.wy
        msg.twist.twist.angular.z = self.wz
        return msg