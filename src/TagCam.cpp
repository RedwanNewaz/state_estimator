//
// Created by redwan on 8/28/22.
//

#include "mjpeg_cam_nodelet/TagCam.h"

namespace mjpeg_cam {
    TagCam::TagCam(ros::NodeHandle &nodeHandle) : MjpegCam(nodeHandle) {

        pub_camera_info_ = false;
        pub_static_transform_ = false;
        try {
            if(m_nodeHandle.hasParam("camera_matrix/data"))
                readCamInfo();
            if(m_nodeHandle.hasParam("static_transform"))
                readStaticTransform();
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        if(pub_camera_info_)
        {
            camInfoPub_ = m_nodeHandle.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
            if(pub_static_transform_)
                ROS_INFO("Successfully Camera node launched with (camera info + static transformation).");
            else
                ROS_INFO("Successfully Camera node launched with camera info.");
        }
        else
            ROS_WARN("Camera info is not found !!");

    }

    void TagCam::timerCallback(const ros::TimerEvent &event) {
        if (m_nodeHandle.ok() && !readAndPublishImage())
            ROS_WARN("Could not publish image");

        if(pub_camera_info_)
        {
            cam_info_msg_.header.stamp = m_img_pub_time;
            camInfoPub_.publish(cam_info_msg_);
        }

        if(pub_static_transform_)
        {

            br_.sendTransform(tf::StampedTransform(transform_, m_img_pub_time, child_frame_, m_camera_frame_id));
        }

    }



    void TagCam::readCamInfo() {

        int img_height, img_width;
        std::vector<double>D, R, P, K;
        std::string distortion_model;
        m_nodeHandle.getParam("image_height", img_height);
        m_nodeHandle.getParam("image_width", img_width);
        m_nodeHandle.getParam("camera_matrix/data", K);
        m_nodeHandle.getParam("distortion_coefficients/data", D);
        m_nodeHandle.getParam("rectification_matrix/data", R);
        m_nodeHandle.getParam("projection_matrix/data", P);
        m_nodeHandle.getParam("distortion_model", distortion_model);
        ROS_INFO("[CameraInfo] image size %d x %d", img_width, img_height);
        cam_info_msg_.width = img_width;
        cam_info_msg_.height = img_height;
        cam_info_msg_.distortion_model = distortion_model;

        cam_info_msg_.header.frame_id = m_camera_frame_id;

        memcpy(&cam_info_msg_.K, K.data(), K.size() * sizeof (double) );
        std::copy(D.begin(), D.end(), std::back_inserter(cam_info_msg_.D));
        memcpy(&cam_info_msg_.R, R.data(), R.size() * sizeof (double));
        memcpy(&cam_info_msg_.P, P.data(), P.size() * sizeof (double));

        pub_camera_info_ = true;
    }

    void TagCam::readStaticTransform() {

        std::vector<double> pos, ori;
        m_nodeHandle.getParam("static_transform/position", pos);
        m_nodeHandle.getParam("static_transform/orientation", ori);
        m_nodeHandle.getParam("static_transform/child_frame", child_frame_);

        assert(pos.size() == 3);
        assert(ori.size() == 4);

        tf::Vector3 translation(pos[0], pos[1], pos[2]);
        tf::Quaternion rotation(ori[0], ori[1], ori[2], ori[3]);
        transform_.setOrigin( translation );
        transform_.setRotation(rotation);
        pub_static_transform_ = true;

    }

} // mjpeg_cam