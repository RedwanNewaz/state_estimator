//
// Created by redwan on 10/31/19.
//

#include "FilterBase.h"


FilterBase::FilterBase() {

    // number of state variables (x,y,phi)
    n = STATE_SIZE;

    // Resizing matrices
    state.resize(n);
    state_new.resize(n);
    P0.resize(n, n); // Initial estimate error covariance
    for (int i = 0; i < n; ++i) {
        P0(i, i) = 0.1;
    }

    dt = 0.03;

}

Eigen::VectorXd FilterBase::get_state() {

    return state;
}

void FilterBase::set(const RobotParams &parm, const std::vector<FieldLocation> &landmark) {

    std::copy(landmark.begin(),landmark.end(),std::back_inserter(true_landmarks_));


    DETECTION_RANGE_ALPHA = parm.sensor_noise_distance;
    DETECTION_ANGLE_SIGMA = parm.sensor_noise_orientation;

//    bad result if we initialize Q and R in here
//    Q(0, 0) = pow(parm.sensor_noise_distance, 2);
//    Q(1, 1) = pow(parm.sensor_noise_orientation, 2);

    ALPHA1 = parm.odom_noise_rotation_from_rotation;
    ALPHA2 = parm.odom_noise_rotation_from_translation;
    ALPHA3 = parm.odom_noise_translation_from_translation;
    ALPHA4 = parm.odom_noise_translation_from_rotation;

//      Don't do it - bad result
//    R(0, 0) = pow(parm.odom_noise_rotation_from_rotation + parm.odom_noise_rotation_from_translation, 2);
//    R(1, 1) = pow(ALPHA3 + ALPHA4, 2);

    FOV = parm.angle_fov * M_PI/180.0;


    // Don't forget to initialize specific filter later !
    initialized_ = false;
}

void FilterBase::render_ellipse() {
    double major, minor, theta;

//    Pxy = PEst[0:2, 0:2]

    Eigen::Matrix<double,2,2> Pxy;
    Pxy = P.block(0, 0, 2, 2);


    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> a(Pxy);
    double e0 = sqrt(a.eigenvalues()(0));
    double e1 = sqrt(a.eigenvalues()(1));

//    printf("(%ld, %ld)\n", e0, e1);

    if (e0 > e1) {
        theta = atan2(a.eigenvectors()(1, 0), a.eigenvectors()(0, 0));
        major = e0;
        minor = e1;
    } else {
        theta = atan2(a.eigenvectors()(1, 1), a.eigenvectors()(0, 1));
        major = e1;
        minor = e0;
    }



}



double FilterBase::constrain_angle(double radian) {
    if (radian < -M_PI) {
        radian += 2*M_PI;
    } else if (radian > M_PI) {
        radian -= 2*M_PI;
    }

    return radian;
}

void FilterBase::measurement_update(const std::vector<MarkerObservation> &landmarks) {
    // given a list of landmark find the one which is detected

    /**
     *  landmarks are given  with respect to robot
     *  so we can easily filter out others because robot has FOV.
     *  Here I have used vector to store information. In some cases
     *  robot might detect two landmarks.
     */

    std::vector<MAP> detected;
    for(auto &l:landmarks)
    {

        if(l.orientation<FOV)
        {
            debug("marker detected "<<l.markerIndex<<" "<<l.orientation);


            detected.push_back(std::make_pair(true_landmarks_[l.markerIndex], l));
        }
        else
            debug(" filtered marker "<<l.orientation);
    }

    landmark_update(detected);


}

