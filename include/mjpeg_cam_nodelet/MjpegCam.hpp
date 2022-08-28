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




protected:
    /*!
     * Reads a single frame from the camera and publish to topic.
     */
    bool readAndPublishImage();


    //! ROS node handle.
    ros::NodeHandle &m_nodeHandle;

    std::string m_camera_frame_id;
    ros::Time m_img_pub_time;



private:

    /*!
     * Reads ROS parameters.
     */
    void readParameters();

    /*!
     * Set camera parameters
     */
    bool setCameraParams();



    //! ROS Image Publisher
    ros::Publisher imagePub_;

    //! Camera Object
    UsbCamera *cam;
    unsigned int sequence;

    // Parameters
    std::string device_name;
    int width;
    int height;
    int framerate;
    int exposure;
    int brightness;
    bool autoexposure;



};

} /* namespace */
