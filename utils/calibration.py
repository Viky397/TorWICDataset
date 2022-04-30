import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

def getStaticTFBaseToCameraOptical(time):
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = 0
    tf_stamped.header.stamp = rospy.Time.from_sec(time)
    tf_stamped.header.frame_id = "base_link"
    tf_stamped.child_frame_id = "realsense_front_camera_color_optical_frame"
    tf_stamped.transform.translation.x = 0.520
    tf_stamped.transform.translation.y = 0.032
    tf_stamped.transform.translation.z = 0.011
    tf_stamped.transform.rotation.x = -0.431
    tf_stamped.transform.rotation.y = 0.429
    tf_stamped.transform.rotation.z = -0.562
    tf_stamped.transform.rotation.w = 0.561
    return tf_stamped

def getStaticTFBaseToIMU(time):
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = 0
    tf_stamped.header.stamp = rospy.Time.from_sec(time)
    tf_stamped.header.frame_id = "base_link"
    tf_stamped.child_frame_id = "imu_link"
    tf_stamped.transform.translation.x = 0.192
    tf_stamped.transform.translation.y = -0.086
    tf_stamped.transform.translation.z = 0.185
    tf_stamped.transform.rotation.x = 0.000
    tf_stamped.transform.rotation.y = 0.000
    tf_stamped.transform.rotation.z = -0.383
    tf_stamped.transform.rotation.w = 0.924
    return tf_stamped

def getStaticTFBaseToLidar(time):
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = 0
    tf_stamped.header.stamp = rospy.Time.from_sec(time)
    tf_stamped.header.frame_id = "base_link"
    tf_stamped.child_frame_id = "front_laser"
    tf_stamped.transform.translation.x = 0.456
    tf_stamped.transform.translation.y = 0.000
    tf_stamped.transform.translation.z = 0.172
    tf_stamped.transform.rotation.x = 0.000
    tf_stamped.transform.rotation.y = 0.000
    tf_stamped.transform.rotation.z = 0.000
    tf_stamped.transform.rotation.w = 1.0
    return tf_stamped

def getStaticTFMsg(time):
    tf_cam = getStaticTFBaseToCameraOptical(time)
    tf_imu = getStaticTFBaseToIMU(time)
    tf_lidar = getStaticTFBaseToLidar(time)

    static_tf_msgs = TFMessage()
    static_tf_msgs.transforms.append(tf_cam)
    static_tf_msgs.transforms.append(tf_imu)
    static_tf_msgs.transforms.append(tf_lidar)

    return static_tf_msgs

def getCameraInfoMsg(id, time, frame):
    cam_info = CameraInfo()
    cam_info.header.seq = id
    cam_info.header.stamp = rospy.Time.from_sec(time)
    cam_info.header.frame_id = "realsense_front_camera_color_optical_frame"
    cam_info.height = 360
    cam_info.width = 640
    cam_info.distortion_model = "plumb_bob"
    cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    cam_info.K = [461.0956115722656, 0.0, 325.4782409667969, 0.0, 460.9181823730469, 177.57662963867188, 0.0, 0.0, 1.0]
    cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info.P = [461.0956115722656, 0.0, 325.4782409667969, 0.0, 0.0, 460.9181823730469, 177.57662963867188, 0.0, 0.0, 0.0, 1.0, 0.0]
        
    return cam_info