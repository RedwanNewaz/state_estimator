# ROS Tag Camera

## Overview

This package captures MJPEG video stream from usb cameras and optionally publish camera info with static transformation. 
Unlike the `usb_cam` package, this driver copies the JPEG data to the `CompressedImage` message directly. 
This is a nodelet implementation of [mjpeg_cam](https://github.com/rsmohamad/mjpeg_cam) but there is no extra overhead from decoding and re-encoding the image. 
So, only compressed images are published and suitable for apriltag-based localization algorithms. 
This driver is suitable for applications where the images are captured by a low power device (e.g. the Raspberry Pi) and sent to a remote node for further processing. 

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- libv4l-dev (for accessing the usb camera)
    ```$xslt
    sudo apt-get install libv4l-dev
    ```
- v4l-utils (for changing camera settings)
    ```$xslt
    sudo apt-get install v4l-utils
