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
    : m_nodeHandle(nodeHandle),
      sequence(0)
{
    readParameters();
    imagePub_ = m_nodeHandle.advertise<sensor_msgs::CompressedImage>("image/compressed", 1);

    cam = new UsbCamera(device_name, width, height);

    try {
        setCameraParams();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }
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
        msg.header.frame_id = m_camera_frame_id;
        msg.header.seq = sequence++;
        m_img_pub_time = ros::Time::now();
        msg.header.stamp =m_img_pub_time;
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
    while (m_nodeHandle.ok()) {
        if (!readAndPublishImage())
            ROS_WARN("Could not publish image");

        loop_rate.sleep();
        ros::spinOnce();
    }
}

void MjpegCam::readParameters()
{
    m_nodeHandle.param("device_name", device_name, std::string("/dev/video0"));
    m_nodeHandle.param("width", width, 640);
    m_nodeHandle.param("height", height, 480);
    m_nodeHandle.param("framerate", framerate, 30);
    m_nodeHandle.param("m_camera_frame_id", m_camera_frame_id, std::string("usb_cam"));
    m_nodeHandle.param("exposure", exposure, 128);
    m_nodeHandle.param("autoexposure", autoexposure, true);
    m_nodeHandle.param("brightness", brightness, 128);


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



} /* namespace */
