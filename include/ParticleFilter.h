//
// Created by redwan on 10/31/19.
//

#ifndef LOCALIZATION_PARTICLEFILTER_H
#define LOCALIZATION_PARTICLEFILTER_H

#include "FilterBase.h"
#include <random>
#include <math.h>

class ParticleFilter : public FilterBase{
public:


    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0);

    /**
    * combine motion and observation models for particle filter
    */

    void predict(const Eigen::VectorXd &dX);

    /**
     * Landmark update based on measured values. The key idea is to memorize the landmark.
     * Update this landmarks when new landmarks detected
     */
    void landmark_update(const std::vector<MarkerObservation>& landmarks);


private:
    const int num_particles_ = 100;
    const int resample_particles = 50;

    // system matrix
    Eigen::Matrix<double, 3, 3> F;
    // input matrix
    Eigen::Matrix<double, 3, 2> B;

    // particles represntation
    std::vector<Eigen::Vector3d> particles_;
    std::vector<double>weights_;

    // landmarks that the robot has seen so far
    std::vector<MarkerObservation> last_seen_landmarks_;
protected:
    Eigen::Vector3d motion_model(const Eigen::Vector3d &x, const Eigen::Vector2d & u);
    double randn();
    /**
     * @param mu mean
     * @param sigma standard deviation
     * @return Gaussina likelihood
     */
    double compute_likelihood(double mu, double sigma);

    // resample particles based on its weight
    void resampling();


};


#endif //LOCALIZATION_PARTICLEFILTER_H
