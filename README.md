# Apriltag-based State Estimator

The keyidea is to use mjpeg camera to get compressed images and then use apriltag to detect robot based on the compressed images. 

# dependency 

clone and build this repo ```https://github.com/rsmohamad/mjpeg_cam```

install these packages 

```
sudo apt install -y ros-noetic-apriltag-ros \
                    ros-noetic-compressed-image-transport \ 
                    ros-noetic-robot-localization  

```