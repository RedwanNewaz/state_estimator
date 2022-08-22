
#ifndef _ROBOT_DEFS_H_
#define _ROBOT_DEFS_H_
#include "Eigen/Dense"

/* Robot state */
const int STATE_SIZE = 3;
struct RobotState 
{
    double x;      // x-position (in meters)
    double y;      // y-position (in meters)
    double theta;  // orientation (in radians)

    static RobotState fromEigenVec(const Eigen::VectorXd &X)
    {
        assert(X.size() == STATE_SIZE);
        return RobotState{X[0], X[1], X[2]};
    }

    Eigen::VectorXd toEigenVec() const
    {
        Eigen::VectorXd X(STATE_SIZE);
        X << x, y, theta;
        return X;
    }
};

/* Field location structure */
struct FieldLocation
{
    double x;      // x-position (in meters)
    double y;      // y-position (in meters)
};

/* Field location structure */
struct MarkerObservation
{
    int markerIndex;    // Index of observed marker [0-3]
    double distance;    // Observed distance to landmark from robot position
    double orientation; // Observed bearing to landmark in local robot coordinate frame
};

/* Robot model parameters */
struct RobotParams 
{
    double angle_fov; // Field of view of robot (in degrees)
    
    // Sensor noise model parameters
    double sensor_noise_distance;
    double sensor_noise_orientation;
    
    // Odometry noise model parameters
    double odom_noise_rotation_from_rotation;
    double odom_noise_rotation_from_translation;
    double odom_noise_translation_from_translation;
    double odom_noise_translation_from_rotation;
};


#endif
