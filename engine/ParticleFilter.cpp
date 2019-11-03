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


    /**************************************************************
     * STEP 3:
     * Compare each observation (by actual vehicle) to corresponding
     * observation by the particle (landmark_in_range)
     * update the particle weight based on this
     **************************************************************/
    double W = 0;
    for(auto &p: particles_)
    {
        Eigen::Vector2d O, M, D, P, S, M_P, O_M;
        P << p.x, p.y;
        S << state(0), state(1);
        double w = 1;
        for (auto &l:landmarks)
        {
            FieldLocation map = l.first;
            M << map.x, map.y;
            M_P = M - P ;
            MarkerObservation obs  = l.second;
            O << obs.distance, obs.orientation;
            O_M << M_P.norm(), atan2(M_P(1), M_P(0)) - p.theta;

            // compute error
            D  = O_M - O;
            w *= compute_likelihood(D(0), D(1));
        }

        this->particles_[p.id].weight = w;
        this->weights_[p.id] = w;
        W += w;

    }

    // normalizing the weights
//    std::transform(weights_.begin(),weights_.end(),weights_.begin(),[&](double val){return val/W;});

    Eigen::VectorXd part;
    part.resize(num_particles_);

    if(fabs(W)<=0)
    for(int i = 0; i<num_particles_; ++i)
    {
        weights_[i] = 1.0/num_particles_*1.0;
//        debug(weights_[i]<<","<<W);
        particles_[i].weight = weights_[i];
        part(i) = weights_[i];
    }
    else
        std::transform(weights_.begin(),weights_.end(),weights_.begin(),[&](double val){return val/W;});

    // FixME - weight becomes infinity and zero
//    double res = std::accumulate(weights_.begin(),weights_.end(),0.0);
//    assert(res == 1.0);
//    std::copy(weights_.begin(),weights_.end(),std::ostream_iterator<double>(std::cout," "));
//    debug(" weights");
//    int resample = 1.0/part.dot(part.transpose());
//    debug("resample "<<resample);
//
//    if(resample<num_particles_/2)
    resampling();

}


void ParticleFilter::resampling() {

    // Resample particles_ with replacement with probability proportional to their weight.
    std::vector<Particle> resampled_particles;
    std::random_device random_device;
    std::mt19937 gen(random_device());
    std::discrete_distribution<int> index(this->weights_.begin(), this->weights_.end());

    for (int c = 0; c < num_particles_; c++) {

        int i = index(gen);

        Particle p {
                i,
                this->particles_[i].x,
                this->particles_[i].y,
                this->particles_[i].theta,
//                sample_x, sample_y, sample_theta,
                1.0// initial weight is 1.0
        };

        resampled_particles.push_back(p);
    }

    this->particles_ = resampled_particles;

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
    // update state vector
//    state_new << best_particle.x, best_particle.y,best_particle.theta;
    state_new(0) = best_particle.x;
    state_new(1) = best_particle.y;
    state_new(2) = best_particle.theta;




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


double ParticleFilter::compute_likelihood(double delta_length, double delta_angle) {

    double std_landmark[] = {DETECTION_RANGE_ALPHA, DETECTION_ANGLE_SIGMA};
    double num_a = delta_length * delta_length / (2.0 * std_landmark[0] * std_landmark[0]);
    double num_b = delta_angle * delta_angle / (2.0 * std_landmark[1] * std_landmark[1]);
    double numerator = exp(-1.0 * (num_a + num_b));
    double denominator = 2.0 * M_PI * std_landmark[0] * std_landmark[1];


    return numerator / denominator;
}

void ParticleFilter::render_ellipse() {

    // show covariance ellipse
    FilterBase::render_ellipse();

    // draw particles

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPointSize(5.0f);
    // points are red
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_POINTS);
    for(auto &p:particles_)
    {
        int xe, ye;
        global2pixel(p.x, p.y, xe, ye);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2f(xe, ye);
    }
    glEnd();
    glPopMatrix();

}
