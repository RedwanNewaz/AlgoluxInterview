//
// Created by redwan on 10/31/19.
//

#ifndef LOCALIZATION_FILTERBASE_H
#define LOCALIZATION_FILTERBASE_H



#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "robot_defs.h"

class FilterBase {
public:
    /**
    * Create a Filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */

    FilterBase();

    /**
    * Return the current state and time.
    */
    Eigen::VectorXd get_state();

    /**
     * set system parameter and landmark
     */
     void set(const RobotParams& parm, const std::vector<FieldLocation>& landmark);


protected:
    /**
     * Matrices for computation (follow up from above definitions)
     * P0 - Initial error covariance
     * P  - Estimated error covariance
     */
    Eigen::MatrixXd P, P0;
    // System dimensions
    int n;
    // Initial and current time
    double t0, t;
    // Discrete time step
    double dt;
    // Is the filter initialized?
    bool initialized;

    // Estimated states
    Eigen::VectorXd state, state_new;
    // global landmarks
    std::vector<FieldLocation> true_landmarks_;

    Eigen::Matrix<double, 2, 2> Q; // measurement noise

    Eigen::Matrix<double, 2, 2> R; // motion noise

    // parameter
    double DETECTION_RANGE_ALPHA, DETECTION_ANGLE_SIGMA;
    double ALPHA1, ALPHA2, ALPHA3, ALPHA4;

    const double EPS = 1e-4;




};

#endif //LOCALIZATION_FILTERBASE_H
