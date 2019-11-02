//
// Created by redwan on 10/31/19.
//

#include "ParticleFilter.h"
#define debug(x) std::cout<<x<<"\n"

void ParticleFilter::init(double t0, const Eigen::VectorXd &x0) {

    state = x0;

    // Landmark measurement uncertainty [x [m], y [m]]
    double sigma_landmark[] = {DETECTION_RANGE_ALPHA, DETECTION_RANGE_ALPHA, DETECTION_ANGLE_SIGMA};

    weights_.resize(num_particles_, 1.0);

    // initial belief- a normal distribution for state
    std::normal_distribution<double> dist_x(state(0), sigma_landmark[0]);
    std::normal_distribution<double> dist_y(state(1), sigma_landmark[1]);
    std::normal_distribution<double> dist_theta(state(2), sigma_landmark[2]);

    // initialize all particles to x0 with measurement uncertainty all all weights to 1

    for (int i = 0; i < num_particles_; ++i) {
        double sample_x, sample_y, sample_theta;
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        Particle particle;
        particle.id = i;
        particle.x = sample_x;
        particle.y = sample_y;
        particle.theta = sample_theta;
        particle.weight = 1.0;

        particles_.push_back(particle);
    }

    P = P0;

    initialized_ = true;

}

void ParticleFilter::predict(const Eigen::VectorXd &delta) {

    // compute control inputs - v, w
    double v = sqrt(delta(0)*delta(0) + delta(1)*delta(1));
    double w = delta(2);

    //Add measurements to each particle and add random Gaussian noise.

    for (auto& p : particles_){

        if (fabs(w) > 0.001) {
            p.x += v/w * (sin(p.theta + w * dt) - sin(p.theta));
            p.y += v/w * (cos(p.theta)  - cos(p.theta + w * dt));
            p.theta  += w * dt;
        }
        else {
            p.x += v * dt * cos(p.theta);
            p.y += v * dt * sin(p.theta);
        }

        // add process noise
        R(0, 0) = pow(ALPHA1*fabs(v) + ALPHA2*fabs(w), 2);
        R(1, 1) = pow(ALPHA3*fabs(v) + ALPHA4*fabs(w), 2);

        std::normal_distribution<double> dist_x(p.x, pow(R(0,0), 0.5));
        std::normal_distribution<double> dist_y(p.y, pow(R(0, 0), 0.5));
        std::normal_distribution<double> dist_theta(p.theta, pow(R(1, 1), 0.5));

        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);

    }

    approximate_state_cov();
    state = state_new;

}

void ParticleFilter::landmark_update(const std::vector<MarkerObservation> &landmarks) {
    // this function is the heart of the particle filter
    double sigma_x, sigma_y;
    sigma_x = sigma_y = DETECTION_RANGE_ALPHA;

    for(int i=0; i < num_particles_; ++i) {

        // Convert detected landmarks
        double probability = 1;
        for(const auto &l : landmarks)
        {
            FieldLocation z = convertVehicleToMapCoords(l, particles_[i]);
            // find the closes one
            double dx = true_landmarks_[l.markerIndex]. x - z.x;
            double dy = true_landmarks_[l.markerIndex]. y - z.y;
            probability *= 1.0/(2*M_PI*sigma_x*sigma_y) * exp(-dx*dx / (2*sigma_x*sigma_x))* exp(-dy*dy / (2*sigma_y*sigma_y));
        }

        particles_[i].weight = probability;
        weights_[i] = probability;
        // Using the converted observations perform data association
    }

    // weights are updated, now perform resampling
    resampling();
    approximate_state_cov();
}


void ParticleFilter::resampling() {

    // Resample particles with replacement with probability proportional to their weight.

    std::discrete_distribution<int> d(weights_.begin(), weights_.end());
    std::vector<Particle> weighted_sample(num_particles_);

    for(int i = 0; i < num_particles_; ++i){
        int j = d(gen);
        weighted_sample.at(i) = particles_.at(j);
    }

    particles_ = weighted_sample;


}

void ParticleFilter::approximate_state_cov() {

    // compute state = best particle
    Particle best_particle;
    double highest_weight = 0.0;
    for (int i = 0; i < num_particles_; ++i) {
        if (particles_[i].weight > highest_weight) {
            highest_weight = particles_[i].weight;
            best_particle = particles_[i];
        }
    }
    // update eigen vector
    state_new << best_particle.x, best_particle.y, best_particle.theta;

    // TODO - compute covariance

    P0.setZero();
    for (int i = 0; i < num_particles_; ++i) {
        Eigen::Vector3d dx;
        dx <<   (particles_[i].x - state_new(0)),
                (particles_[i].y - state_new(1)),
                (particles_[i].theta - state_new(2));
        P0 += weights_[i] * dx*dx.transpose();

    }

    P = P0/num_particles_;

}

FieldLocation ParticleFilter::convertVehicleToMapCoords(const MarkerObservation &marker, const Particle &p) {

    double o_x = marker.distance * cos(p.theta);
    double o_y = marker.distance * sin(p.theta);

    double x = p.x + o_x * cos(p.theta) - o_y * sin(p.theta);
    double y = p.y + o_x * sin(p.theta) - o_y * cos(p.theta);

    return {x, y};

}
