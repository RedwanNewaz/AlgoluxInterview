//
// Created by redwan on 10/31/19.
//

#include "ParticleFilter.h"
#define debug(x) std::cout<<x<<"\n"

void ParticleFilter::init(double t0, const Eigen::VectorXd &x0) {

    state = x0;
    particles_.resize(num_particles_);
    weights_.resize(num_particles_);
    double init_prob = 1.0/double(num_particles_);
    std::fill(weights_.begin(),weights_.end(),init_prob);
    initialized_ = true;

}

void ParticleFilter::predict(const Eigen::VectorXd &delta) {


    double v = sqrt(delta(0)*delta(0) + delta(1)*delta(1));
    double w = delta(2);
    Eigen::Matrix<double, 2, 1> u;
    u <<v,w;

    // TODO - initialize noise vector

    // localization with particle filter
    for(int i = 0; i<num_particles_; ++i)
    {
        Eigen::Vector3d x = particles_[i];
        double w = weights_[i];

        // predicting with random input sampling
        Eigen::Matrix<double, 2, 1> ud;
        // add control noise
        R(0, 0) = pow(ALPHA1*fabs(v) + ALPHA2*fabs(w), 2);
        R(1, 1) = pow(ALPHA3*fabs(v) + ALPHA4*fabs(w), 2);
        double ud1 = u[0, 0] + randn() * pow(R(0, 0), 0.5);
        double ud2 = u[1, 0] + randn() * pow(R(1, 1), 0.5);
        ud << ud1, ud2;
        x = motion_model(x, ud);

        for (const auto &l : last_seen_landmarks_) {

            int index = l.markerIndex;
            double zx = -(true_landmarks_[index].x - state(0))/l.distance;
            double zy = -(true_landmarks_[index].y - state(1))/l.distance;
            double zd = sqrt(pow(zx,2) + pow(zy,2));
            //  Calc Importance Weight
            double dx = x(0) - zx;
            double dy = x(1) - zy;
            double pre_z = sqrt(pow(dx,2) + pow(dy,2));
            double dz = pre_z - zd;

            // add observation noise
            Q(0, 0) = pow(l.distance*DETECTION_RANGE_ALPHA, 2);
            Q(1, 1) = pow(l.orientation*DETECTION_ANGLE_SIGMA, 2);

            // update weight
            w *= compute_likelihood(dz, sqrt(Q(0, 0)));

        }

        // update tracking
        particles_[i] = x;
        weights_[i] = w;

    }

    // normalize weight
    double weight_sum = std::accumulate(weights_.begin(),weights_.end(),0.0);
    std::transform(weights_.begin(),weights_.end(),weights_.begin(),[&](double w){ return (w/weight_sum);});


//    debug("new state \n"<< state_new);
    // TODO approximate state and compute covaraince
//    state_new.setZero();
    for(int i = 0; i<num_particles_; ++i)
    {
        state_new += weights_[i]*particles_[i];

    }
    state_new /= num_particles_;
    // estimate state


    // calculate covariance
//    for i in range(px.shape[1]):
//    dx = (px[:, i] - xEst)[0:3]
//    cov += pw[0, i] * dx.dot(dx.T)
//    cov /= NP


// update state
state = state_new;

}

void ParticleFilter::landmark_update(const std::vector<MarkerObservation> &landmarks) {

    // TODO - provide thread safety
    last_seen_landmarks_.clear();
    std::copy(landmarks.begin(),landmarks.end(), std::back_inserter(last_seen_landmarks_));

}

Eigen::Vector3d ParticleFilter::motion_model(const Eigen::Vector3d &x, const Eigen::Vector2d &u) {
    F.setIdentity();
    B <<dt * cos(x(2)), 0,
        dt * sin(x(2)), 0,
        0,   dt;
//    python: new_state =  F.dot(state) + B.dot(u);
    return F.adjoint()*x  + B*u;
}

double ParticleFilter::randn() {
    unsigned int seed = 2019;
    std::mt19937 generator; // it doesn't matter if I use the std::mt19937_64 or the std::mt19937
    generator.seed(seed);
    std::normal_distribution<double> normal; // default is 0 mean and 1.0 std
    return normal(generator);

}

double ParticleFilter::compute_likelihood(double mu, double sigma) {

    sigma = pow(sigma, 2); // sigma is now variance
    double prob = 1.0 / sqrt(2.0 * M_PI * sigma ) * exp(- pow(mu, 2) / (2 * sigma ));
    return prob;
}

void ParticleFilter::resampling() {

    w_cum = np.cumsum(pw)
    base = np.arange(0.0, 1.0, 1/NP)
    re_sample_id = base + np.random.uniform(0, 1/NP)
    indexes = []
    ind = 0
    for ip in range(NP):
    while re_sample_id[ip] > w_cum[ind]:
    ind += 1
    indexes.append(ind)

    px = px[:, indexes]
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

}
