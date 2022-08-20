#!/usr/bin/python3
import sys, yaml, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# Ros libraries
import roslib
import rospy, tf

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/usb_cam/image_raw",
                                         Image)
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/mjpeg_cam/image/compressed",
                                           CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            rospy.loginfo("subscribed to /mjpeg_cam/image/compressed")

        self.camera_calibration_path = '/home/redwan/.ros/camera_info/head_camera.yaml'
        self.camera_param_msg = self.build_camera_info(self.camera_calibration_path)
        rospy.loginfo(self.camera_param_msg)

        self.camera_info_pub = rospy.Publisher("/usb_cam/camera_info", CameraInfo)



    def handle_camera_pose(self, timestamp):
        # x: 2.3342775, y: -0.1338066, z: -0.1595741
        self.br.sendTransform((-0.3075, -0.9038, 3.6429),
                              ( 0.91674, 0.046957, -0.09252, 0.385756 ),
                              timestamp,
                              "map",
                              "usb_cam"
                         )


    def build_camera_info(self, yaml_fname):  # pylint: disable=no-self-use
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle, Loader=yaml.SafeLoader)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg



    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            rospy.loginfo('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:


        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")

        # Publish new image
        timestamp = rospy.Time.now()
        msg.header.stamp = timestamp
        msg.header.frame_id = "usb_cam"

        self.image_pub.publish(msg)

        #self.subscriber.unregister()
        self.camera_param_msg.header.stamp = timestamp
        self.camera_param_msg.header.frame_id = "usb_cam"
        self.camera_info_pub.publish(self.camera_param_msg)

        self.handle_camera_pose(timestamp)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('compressed_image_to_image_raw', anonymous=True, disable_signals=True)
    time.sleep(2)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)