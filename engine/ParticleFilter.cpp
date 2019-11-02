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

    // initialize all particles_ to x0 with measurement uncertainty all all weights to 1

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
    P0 = P;

}

void ParticleFilter::landmark_update(const std::vector<MAP> &landmarks) {


        // weights are updated, now perform resampling
//    resampling();
//    approximate_state_cov();
}


void ParticleFilter::resampling() {

    // Resample particles_ with replacement with probability proportional to their weight.

//    std::discrete_distribution<int> d(weights_.begin(), weights_.end());
//    std::vector<Particle> new_particles_(num_particles_);
//
//    for(int i = 0; i < num_particles_; ++i){
//        int j = d(gen);
//        new_particles_.at(i) = particles_.at(j);
//    }
//
//    particles_ = weighted_sample;


    std::vector<Particle> new_particles_;

    // get all of the current weights
    std::vector<double> weights;
    for (int i = 0; i < num_particles_; i++) {
        weights.push_back(particles_[i].weight);
    }

    // generate random starting index for resampling wheel
    std::uniform_int_distribution<int> uniintdist(0, num_particles_-1);
    auto index = uniintdist(gen);

    // get max weight
    double max_weight = *max_element(weights.begin(), weights.end());

    // uniform random distribution [0.0, max_weight)
    std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    // spin the resample wheel!
    for (int i = 0; i < num_particles_; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles_;
        }
        new_particles_.push_back(particles_[index]);
    }

//    particles_ = new_particles_;
    particles_.clear();
    std::copy(new_particles_.begin(),new_particles_.end(),std::back_inserter(particles_));

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
    state_new << best_particle.x, best_particle.y,
            constrain_angle(best_particle.theta);

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

    double x,y;
    double r = marker.distance;
    double q = marker.orientation;
    x = p.x - r*sin(p.theta) + r*sin(q+p.theta);
    y = p.y + r*cos(p.theta) - r*cos(q+p.theta);

    return {x,y};

}

void ParticleFilter::dataAssociation(const std::vector<std::pair<int, FieldLocation>> &predicted,
                                     std::vector<std::pair<int, FieldLocation>> &observations) {

    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    for (unsigned int i = 0; i < observations.size(); i++) {

        // grab current observation
        FieldLocation o = observations[i].second;

        // init minimum distance to maximum possible
        double min_dist = std::numeric_limits<double>::max();

        // init id of landmark from map placeholder to be associated with the observation
        int map_id = -1;

        for (unsigned int j = 0; j < predicted.size(); j++) {
            // grab current prediction
            FieldLocation p = predicted[j].second;

            // get distance between current/predicted landmarks
            auto dist = [](double x1, double y1, double x2, double y2) {
                return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            };
            double cur_dist = dist(o.x, o.y, p.x, p.y);

            // find the predicted landmark nearest the current observed landmark
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                map_id = predicted[j].first;
            }
        }

        // set the observation's id to the nearest predicted landmark's id
        observations[i].first = map_id;
    }

}
