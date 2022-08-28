//
// Created by redwan on 8/28/22.
//
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "mjpeg_cam_nodelet/MjpegCam.hpp"

namespace mjpeg_cam_nodelet
{

    class CompressedImage : public nodelet::Nodelet
    {
    public:
        CompressedImage()
        {}
        virtual ~CompressedImage(){
            delete mjpegCam_;
        }

    private:
        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            mjpegCam_ = new mjpeg_cam::MjpegCam(private_nh);
            int framerate;
            private_nh.param("framerate", framerate, 30);
            timer_ = private_nh.createTimer(ros::Duration(1.0 / (double ) framerate), &mjpeg_cam::MjpegCam::timerCallback, mjpegCam_);
        }



        ros::Timer timer_;
        mjpeg_cam::MjpegCam *mjpegCam_;


    };

    PLUGINLIB_EXPORT_CLASS(mjpeg_cam_nodelet::CompressedImage, nodelet::Nodelet)
}