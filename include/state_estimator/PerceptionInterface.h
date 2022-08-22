//
// Created by redwan on 8/20/22.
//

#ifndef STATE_ESTIMATOR_PERCEPTIONINTERFACE_H
#define STATE_ESTIMATOR_PERCEPTIONINTERFACE_H

#include <ros/ros.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>



#include <unordered_map>
#include <memory>
#include <cmath>
#include "robot_defs.h"
#include "EKF.h"


using namespace std;

class PerceptionInterface {

public:
    PerceptionInterface();

private:
    ros::NodeHandle nh_;
    ros::Subscriber apriltagSub_;
    unordered_map<int, FieldLocation> landmarks_;
    bool initialization_;
    EKF ekf_;
    ros::Timer timer_;
    RobotState previousState_;
    RobotParams params_;
protected:
    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);

};


#endif //STATE_ESTIMATOR_PERCEPTIONINTERFACE_H
