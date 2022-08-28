#include "mjpeg_cam_nodelet/MjpegCam.hpp"
#include <cassert>

namespace mjpeg_cam
{

void clamp(int &val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
}

MjpegCam::MjpegCam(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle),
      sequence(0)
{
    readParameters();
    imagePub_ = nodeHandle_.advertise<sensor_msgs::CompressedImage>("image/compressed", 1);

    cam = new UsbCamera(device_name, width, height);
    pub_camera_info = false;
    pub_static_transform = false;
    try {
        setCameraParams();
        if(nodeHandle_.hasParam("camera_matrix/data"))
            readCamInfo();
        if(nodeHandle_.hasParam("static_transform"))
            readStaticTransform();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }
    if(pub_camera_info)
    {
        camInfoPub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
        if(pub_static_transform)
            ROS_INFO("Successfully Camera node launched with (camera info + static transformation).");
        else
            ROS_INFO("Successfully Camera node launched with camera info.");
    }
    else
        ROS_WARN("Camera info is not found !!");


}

MjpegCam::~MjpegCam()
{
    delete cam;
}

bool MjpegCam::readAndPublishImage()
{
    try {
        int length;
        char *image = cam->grab_image(length);
        sensor_msgs::CompressedImage msg;
        msg.header.frame_id = camera_frame_id;
        msg.header.seq = sequence++;
        msg.header.stamp = img_pub_time = ros::Time::now();
        msg.format = "jpeg";
        msg.data.resize(length);
        std::copy(image, image + length, msg.data.begin());
        imagePub_.publish(msg);
        //std::cout << "Image size in kB: " << length/1000 << std::endl;

        return true;
    }
    catch (const char *e) {
        std::cout << e << std::endl;
    }

    return false;
}
void MjpegCam::spin()
{
    ros::Rate loop_rate(framerate);
    while (nodeHandle_.ok()) {
        if (!readAndPublishImage())
            ROS_WARN("Could not publish image");

        loop_rate.sleep();
        ros::spinOnce();
    }
}

void MjpegCam::readParameters()
{
    nodeHandle_.param("device_name", device_name, std::string("/dev/video0"));
    nodeHandle_.param("width", width, 640);
    nodeHandle_.param("height", height, 480);
    nodeHandle_.param("framerate", framerate, 30);
    nodeHandle_.param("camera_frame_id", camera_frame_id, std::string("usb_cam"));
    nodeHandle_.param("exposure", exposure, 128);
    nodeHandle_.param("autoexposure", autoexposure, true);
    nodeHandle_.param("brightness", brightness, 128);


}

bool MjpegCam::setCameraParams()
{
    if (cam == 0)
        return false;

    clamp(exposure, 0, 255);
    clamp(brightness, 0, 255);

    cam->set_v4l2_param("brightness", brightness);

    if (autoexposure) {
        cam->set_v4l2_param("exposure_auto", 3);
    }
    else {
        cam->set_v4l2_param("exposure_auto", 1);
        cam->set_v4l2_param("exposure_absolute", exposure);
    }


    return true;
}

void MjpegCam::setDynamicParams(int exposure, int brightness, bool autoexposure)
{
    this->exposure = exposure;
    this->brightness = brightness;
    this->autoexposure = autoexposure;
    setCameraParams();
}

void MjpegCam::timerCallback(const ros::TimerEvent &event) {
    if (nodeHandle_.ok() && !readAndPublishImage())
            ROS_WARN("Could not publish image");

    if(pub_camera_info)
    {
        cam_info_msg.header.stamp = img_pub_time;
        camInfoPub_.publish(cam_info_msg);
    }

    if(pub_static_transform)
    {

        br.sendTransform(tf::StampedTransform(transform, img_pub_time, child_frame, camera_frame_id));
    }

}

void MjpegCam::readCamInfo() {

    int img_height, img_width;
    std::vector<double>D, R, P, K;
    std::string distortion_model;
    nodeHandle_.getParam("image_height", img_height);
    nodeHandle_.getParam("image_width", img_width);
    nodeHandle_.getParam("camera_matrix/data", K);
    nodeHandle_.getParam("distortion_coefficients/data", D);
    nodeHandle_.getParam("rectification_matrix/data", R);
    nodeHandle_.getParam("projection_matrix/data", P);
    nodeHandle_.getParam("distortion_model", distortion_model);
    ROS_INFO("[CameraInfo] image size %d x %d", img_width, img_height);
    cam_info_msg.width = width;
    cam_info_msg.height = height;
    cam_info_msg.distortion_model = distortion_model;

    cam_info_msg.header.frame_id = camera_frame_id;

    memcpy(&cam_info_msg.K, K.data(), K.size() * sizeof (double) );
    std::copy(D.begin(), D.end(), std::back_inserter(cam_info_msg.D));
    memcpy(&cam_info_msg.R, R.data(), R.size()* sizeof (double));
    memcpy(&cam_info_msg.P, P.data(), P.size()* sizeof (double));

    pub_camera_info = true;
}

void MjpegCam::readStaticTransform() {

    std::vector<double> pos, ori;
    nodeHandle_.getParam("static_transform/position", pos);
    nodeHandle_.getParam("static_transform/orientation", ori);
    nodeHandle_.getParam("static_transform/child_frame", child_frame);

    assert(pos.size() == 3);
    assert(ori.size() == 4);

    tf::Vector3 translation(pos[0], pos[1], pos[2]);
    tf::Quaternion rotation(ori[0], ori[1], ori[2], ori[3]);
    transform.setOrigin( translation );
    transform.setRotation(rotation);
    pub_static_transform = true;

}

} /* namespace */
