//
// Created by redwan on 8/20/22.
//
#include <ros/ros.h>
#include "PerceptionInterface.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "moving_cam_localizer");
    ros::NodeHandle nh;

    PerceptionInterface perception;

    ros::spin();
    return 0;
}