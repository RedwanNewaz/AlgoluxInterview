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

    dt = 1;

}

Eigen::VectorXd FilterBase::get_state() {

    return state;
}

void FilterBase::set(const RobotParams &parm, const std::vector<FieldLocation> &landmark) {

    std::copy(landmark.begin(),landmark.end(),std::back_inserter(true_landmarks_));


    DETECTION_RANGE_ALPHA = parm.sensor_noise_distance;
    DETECTION_ANGLE_SIGMA = parm.sensor_noise_orientation;

//    Q(0, 0) = pow(parm.sensor_noise_distance, 2);
//    Q(1, 1) = pow(parm.sensor_noise_orientation, 2);

    ALPHA1 = parm.odom_noise_rotation_from_rotation;
    ALPHA2 = parm.odom_noise_rotation_from_translation;
    ALPHA3 = parm.odom_noise_translation_from_translation;
    ALPHA4 = parm.odom_noise_translation_from_rotation;

//
//    R(0, 0) = pow(parm.odom_noise_rotation_from_rotation + parm.odom_noise_rotation_from_translation, 2);
//    R(1, 1) = pow(ALPHA3 + ALPHA4, 2);


    // FIXME assuming discrete time update happens
//    dt = 1;
    // Don't forget to initialize specific filter later !
    initialized = false;
}

