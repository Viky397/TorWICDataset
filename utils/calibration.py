import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

def getStaticTFBaseToCameraOptical(id, time):
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = id
    tf_stamped.header.stamp = rospy.Time.from_sec(time)
    tf_stamped.header.frame_id = "base_link"
    tf_stamped.child_frame_id = "realsense_front_camera_color_optical_frame"
    tf_stamped.transform.translation.x = 0.12944592
    tf_stamped.transform.translation.y = 0.04299934
    tf_stamped.transform.translation.z = -0.11374339 
    tf_stamped.transform.rotation.x = -0.61167248
    tf_stamped.transform.rotation.y = 0.39292797
    tf_stamped.transform.rotation.z = -0.3567415
    tf_stamped.transform.rotation.w = 0.5866855
    return tf_stamped

def getStaticTFMsg(id, time):
    tf_cam = getStaticTFBaseToCameraOptical(id, time)

    static_tf_msgs = TFMessage()
    static_tf_msgs.transforms.append(tf_cam)

    return static_tf_msgs

def getCameraInfoMsg(id, time, frame):
    cam_info = CameraInfo()
    cam_info.header.seq = id
    cam_info.header.stamp = rospy.Time.from_sec(time)
    cam_info.header.frame_id = "realsense_front_camera_color_optical_frame"
    cam_info.height = 720
    cam_info.width = 1280
    cam_info.distortion_model = "plumb_bob"

    cam_info.D = [0, 0, 0, 0, 0]
    cam_info.K = [621.397474650905, 0.0, 649.644481364277, 0.0, 620.649424677401, 367.908146431575, 0.0, 0.0, 1.0]
    cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info.P = [621.397474650905, 0.0, 649.644481364277, 0.0, 0.0, 620.649424677401, 367.908146431575, 0.0, 0.0, 0.0, 1.0, 0.0]

    return cam_info