//
// Created by redwan on 10/31/19.
//

#ifndef LOCALIZATION_PARTICLEFILTER_H
#define LOCALIZATION_PARTICLEFILTER_H

#include "FilterBase.h"
#include <random>
#include <math.h>
#include <numeric>
#include <algorithm>

struct Particle{
    int id ;
    double x, y, theta;
    double weight;
};

static std::default_random_engine gen;

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
    void landmark_update(const std::vector<MAP>& landmarks);


private:
    const int num_particles_ = 300;
    const int resample_particles = 50;

    // system matrix
    Eigen::Matrix<double, 3, 3> F;
    // input matrix
    Eigen::Matrix<double, 3, 2> B;

    // particles represntation
    std::vector<Particle> particles_;
    std::vector<double>weights_;

    // landmarks that the robot has seen so far
    std::vector<MarkerObservation> last_seen_landmarks_;
protected:
    /**
     * Calculate and output the average weighted error of the particle filter
     * all time steps so far. Then compute error
     */
    void approximate_state_cov();
    /**
     * @param mu mean
     * @param sigma standard deviation
     * @return Gaussina likelihood
     */
//    double compute_likelihood(double mu, double sigma);

    /**
     * resample particles based on its weight
     */
    void resampling();

    FieldLocation convertVehicleToMapCoords(const MarkerObservation &marker, const Particle &p );

    void dataAssociation(const std::vector<std::pair<int, FieldLocation>>& predicted, std::vector<std::pair<int, FieldLocation>>& observations);


};


#endif //LOCALIZATION_PARTICLEFILTER_H
