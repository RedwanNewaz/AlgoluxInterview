//
// Created by redwan on 10/31/19.
//

#ifndef LOCALIZATION_EKF_H
#define LOCALIZATION_EKF_H

#include "FilterBase.h"
#include <vector>
#include <math.h>

class EKF : public FilterBase{
public:

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0);


    /**
    * Prediction with EKF
    */

    void predict(const Eigen::VectorXd &dX);

    /**
     * Landmark update based on measured values. The time step is assumed to remain constant.
     */
     void landmark_update(const std::vector<MarkerObservation>& landmarks);

private:
    Eigen::Matrix<double, 3, 3> G; // Jacobian (motion update / state variable)
    Eigen::Matrix<double, 3, 2> V; // Jacobian (motion noise / state variable)


    Eigen::Matrix<double, 2, 1> ZHAT; // expected sensor measurement
    Eigen::Matrix<double, 2, 1> Z; // sensor measurement
    Eigen::Matrix<double, 2, 3> H; // Jacobian
    Eigen::Matrix<double, 2, 2> S;

    Eigen::Matrix<double, 3, 2> K; // Kalman gain
    Eigen::Matrix<double, 3, 3> I; // Identity

    std::vector<MarkerObservation> last_seen_landmarks_;


protected:
    void nonZeroAngularVelocity(double v, double w, double theta);
    void zeroAngularVelocity(double v, double w, double theta);
    double constrain_angle(double radian);





};



#endif //LOCALIZATION_EKF_H
