//
// Created by redwan on 8/28/22.
//

#ifndef MJPEG_CAM_NODELET_TAGCAM_H
#define MJPEG_CAM_NODELET_TAGCAM_H
#include "MjpegCam.hpp"

namespace mjpeg_cam {

    class TagCam : public MjpegCam{

    public:
        explicit TagCam(ros::NodeHandle &nodeHandle);

        void timerCallback(const ros::TimerEvent& event);

    private:
        /*!
         * Read Camera parameter
         */
        void readCamInfo();

        /*!
        * Read Static transform
        */
        void readStaticTransform();

        // camera info
        ros::Publisher camInfoPub_;

        bool pub_camera_info_;
        sensor_msgs::CameraInfo cam_info_msg_;

        // transformation
        bool pub_static_transform_;
        tf::Transform transform_;
        tf::TransformBroadcaster br_;
        std::string child_frame_;



    };

} // mjpeg_cam

#endif //MJPEG_CAM_NODELET_TAGCAM_H
