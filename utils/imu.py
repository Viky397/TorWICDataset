import rospy
from sensor_msgs.msg import Imu
from time_util import toSec

# Wrapper for IMU data
class IMU:
    def __init__(self, id, data = [0 for i in range(8)]):
        assert(len(data) == 8)

        # Frame
        self.frame = "imu_link"
        # Topic
        self.topic = "/imu/data"
        # ID
        self.id = id

        # Timestamp
        self.sec = int(data[0])
        self.nsec = int(data[1])
        self.time = toSec(self.sec, self.nsec)

        # Linear acceleration
        self.ax = float(data[2])
        self.ay = float(data[3])
        self.az = float(data[4])

        # Angular velocity
        self.wx = float(data[5])
        self.wy = float(data[6])
        self.wz = float(data[7])

    def toROSMsg(self):
        msg = IMU()
        msg.header.seq = self.id
        msg.header.stamp = rospy.Time.from_sec(self.time)
        msg.header.frame_id = self.frame
        msg.child_frame_id = self.child_frame
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az
        msg.angular_velocity.x = self.wx
        msg.angular_velocity.y = self.wy
        msg.angular_velocity.z = self.wz
        return msg