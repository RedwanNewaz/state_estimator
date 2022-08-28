#pragma once

#include "mjpeg_cam_nodelet/MjpegCam.hpp"
#include "mjpeg_cam_nodelet/UsbCamera.hpp"
#include "sensor_msgs/CameraInfo.h"
#include <tf/transform_broadcaster.h>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/CompressedImage.h>


namespace mjpeg_cam
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class MjpegCam
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    MjpegCam(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~MjpegCam();

    /*!
     * Enters an event loop to read the camera
     */
    void spin();

    /*!
     * Set parameters that can be dynamically reconfigured
     */
    void setDynamicParams(int exposure, int brightness, bool autoexposure);


    void timerCallback(const ros::TimerEvent& event);

private:
    /*!
     * Reads a single frame from the camera and publish to topic.
     */
    bool readAndPublishImage();

    /*!
     * Reads ROS parameters.
     */
    void readParameters();

    /*!
     * Read Camera parameter
     */
     void readCamInfo();

    /*!
    * Read Static transform
    */
    void readStaticTransform();

    /*!
     * Set camera parameters
     */
    bool setCameraParams();

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS Image Publisher
    ros::Publisher imagePub_, camInfoPub_;

    //! Camera Object
    UsbCamera *cam;
    unsigned int sequence;

    // Parameters
    std::string device_name, camera_frame_id;
    int width;
    int height;
    int framerate;
    int exposure;
    int brightness;
    bool autoexposure;

    // camera info
    ros::Time img_pub_time;
    bool pub_camera_info;
    sensor_msgs::CameraInfo cam_info_msg;

    // transformation
    bool pub_static_transform;
    tf::Transform transform;
    tf::TransformBroadcaster br;
    std::string child_frame;

};

} /* namespace */
