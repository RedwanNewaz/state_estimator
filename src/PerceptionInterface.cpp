//
// Created by redwan on 8/20/22.
//

#include "PerceptionInterface.h"

PerceptionInterface::PerceptionInterface() {
    apriltagSub_ = nh_.subscribe("/tag_detections", 10, &PerceptionInterface::apriltagCallback, this);


    params_.angle_fov = 25.0;
    params_.sensor_noise_distance = 0.05;
    params_.sensor_noise_orientation = 0.35;
    params_.odom_noise_rotation_from_rotation =  0.2;
    params_.odom_noise_rotation_from_translation =  0.2;
    params_.odom_noise_translation_from_translation =  0.2;
    params_.odom_noise_translation_from_rotation =  0.2;

    // TODO configure FieldLocation of the marker


    landmarks_[0] = FieldLocation{0, 0};

    std::vector<FieldLocation> landmarks;
    for(auto it : landmarks_)
        landmarks.emplace_back(it.second);
    ekf_.set(params_, landmarks);
    timer_ = nh_.createTimer(ros::Duration(0.03), &PerceptionInterface::timerCallback, this);
    initialization_ = false;
}

void PerceptionInterface::apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

//    ROS_INFO_STREAM(msg);
    std::vector<MarkerObservation> observations;

    for (int i = 0; i < msg->detections.size(); ++i) {
        auto tag = msg->detections[i];
        MarkerObservation marker{};
        marker.markerIndex = tag.id[0];

        if(landmarks_.find(marker.markerIndex) != landmarks_.end())
        {
            // compute distance
            auto coord = tag.pose.pose.pose.position;
            marker.distance = sqrt(coord.x * coord.x + coord.y * coord.y);

            // compute orientation
            tf2::Quaternion quat_tf;
            geometry_msgs::Quaternion quat_msg =  tag.pose.pose.pose.orientation;
            tf2::fromMsg(quat_msg, quat_tf);
            marker.orientation = fmod(quat_tf.getAngle() + atan2(coord.y, coord.x) + M_PI, 2 * M_PI) - M_PI;


            FieldLocation fieldCoord{coord.x, coord.y};

            Eigen::VectorXd x(STATE_SIZE);
            x << coord.x, coord.y, quat_tf.getAngle();
            if(!initialization_)
            {
                ekf_.init(msg->header.stamp.toNSec(), x);
                initialization_ = true;
                ROS_INFO_STREAM("Perception system initialized !!!");
                timer_.start();

            }
            else
            {
                auto obs_map = make_pair(fieldCoord, marker);
                observations.push_back(marker);
            }
        }

    }/******FOR loop terminate here *******/

    // update EKF
    if(!observations.empty())
    {
        ekf_.measurement_update(observations);
    }

    ekf_.update_time(msg->header.stamp.toNSec());




}

void PerceptionInterface::timerCallback(const ros::TimerEvent& event) {

    if(!initialization_)return;


    Eigen::VectorXd X = ekf_.get_state();

//    ROS_INFO("sensor update timeout = %lf", ekf_.timeLag());
//    ROS_INFO_STREAM(X.transpose());

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(X[0], X[1], 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, X[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "usb_cam", "ekf_robot"));


}
