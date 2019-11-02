//
// Created by redwan on 10/31/19.
//

#ifndef LOCALIZATION_FILTERBASE_H
#define LOCALIZATION_FILTERBASE_H



#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <GL/glu.h>
#include "robot_defs.h"
#include "controller.h"

#define debug(x) std::cout<< "[FilterBase]:: " <<x<<"\n";
typedef std::pair<FieldLocation, MarkerObservation> MAP;

class FilterBase {
public:
    /**
    * Create a Filter with the specified matrices.
    *   G - System dynamics matrix
    *   V - Output matrix
    *   R - Process noise covariance
    *   Q - Measurement noise covariance
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

     void render_ellipse();

     void measurement_update(const std::vector<MarkerObservation>& landmarks);

     virtual void landmark_update(const std::vector<MAP>& landmarks) = 0;

//     virtual void landmark_updates(const std::vector<MarkerObservation>& landmarks) = 0;


protected:
    /**
     * Matrices for computation (follow up from above definitions)
     * P0 - Initial error covariance
     * P  - Estimated error covariance
     */

    Eigen::MatrixXd P, P0;

    // State dimensions
    int n;
    // Discrete time step
    double dt;
    // Is the filter initialized_?
    bool initialized_;

    // Estimated states
    Eigen::VectorXd state, state_new;
    // global landmarks
    std::vector<FieldLocation> true_landmarks_;

    Eigen::Matrix<double, 2, 2> Q; // measurement noise

    Eigen::Matrix<double, 2, 2> R; // motion noise

    // sensor noise parameters
    double DETECTION_RANGE_ALPHA, DETECTION_ANGLE_SIGMA;
    // process noise paramters
    double ALPHA1, ALPHA2, ALPHA3, ALPHA4;
    // threshold for detecting zero angular velocity
    const double EPS = 1e-4;

    // double detection FOV
    double FOV;

    double constrain_angle(double radian);




};

#endif //LOCALIZATION_FILTERBASE_H
