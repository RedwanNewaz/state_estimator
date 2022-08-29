//
// Created by redwan on 8/28/22.
//
#include <ros/ros.h>
#include "mjpeg_cam_nodelet/TagCam.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tag_cam_node");
    ros::NodeHandle nh("~");
    mjpeg_cam::TagCam *tagCam = new mjpeg_cam::TagCam(nh);
    int framerate;
    nh.param("framerate", framerate, 30);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / (double ) framerate), &mjpeg_cam::TagCam::timerCallback, tagCam);
    ros::spin();
    delete tagCam;
    return 0;
}