#!/usr/bin/python3
import sys, time

import rospy, tf

# Ros Messages
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class TagToNavMsg:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.br = tf.TransformBroadcaster()
        # topic where we publish
        self.nav_pub = rospy.Publisher("/robot/camera_pose",
                                       PoseWithCovarianceStamped, queue_size=1)


        # subscribed Topic
        self.subscriber = rospy.Subscriber("/tag_detections",
                                           AprilTagDetectionArray, self.callback,  queue_size = 1)

    def handle_robot_pose(self, pose, frame_id, timestamp):

        self.br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                              ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ),
                              timestamp,
                              frame_id,
                              "robot")

    def toNavMsg(self, frame_id, pose):
        msg = PoseWithCovarianceStamped()

        msg.pose.pose = pose
        msg.header.stamp = timestamp = rospy.Time.now()
        msg.header.frame_id = "usb_cam"
        # self.handle_robot_pose(pose, frame_id, timestamp)
        # msg.header. = 'robot'
        return msg

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        for tag in ros_data.detections:
            tag_id = tag.id[0]
            pose = tag.pose.pose.pose
            # print(tag)
            msg = self.toNavMsg(tag.pose.header.frame_id, pose)
            self.nav_pub.publish(msg)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('april_tag_to_navigation_msg', anonymous=True, disable_signals=True)
    time.sleep(2)
    print('april tag to navigation msg initiated')
    tag = TagToNavMsg()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")


if __name__ == '__main__':
    main(sys.argv)