//
// Created by redwan on 10/31/19.
//

#include "EKF.h"
#include <cassert>
#define debug(x) std::cout<< x <<std::endl;

void EKF::init(double t0, const Eigen::VectorXd &x0) {

    // initialize identity matrix

    // robot initial state
    state = x0;
    // update covariance
    P = P0;

    Q.setZero();
    H.setZero();
    I.setIdentity();


    G.setIdentity();
    V.setZero();
    R.setZero();

    initialized_ = true;


}


void EKF::predict(const Eigen::VectorXd &dX) {

    assert(initialized_ && "EKF is not intialized");


    // convert delta to control inputs which are v (linear velocity) and omega (angular velocity)

    double v = sqrt(dX(0)*dX(0) + dX(1)*dX(1));
    double w = dX(2);
    double theta = state(2);
//
//    // noise
    R(0, 0) = pow(ALPHA1*fabs(v) + ALPHA2*fabs(w), 2);
    R(1, 1) = pow(ALPHA3*fabs(v) + ALPHA4*fabs(w), 2);

    if(fabs(w) > EPS)
        nonZeroAngularVelocity(v,w,theta);
    else
        zeroAngularVelocity(v,w,theta);

    state = state_new;
    P0 = P;


}

void EKF::landmark_update(const std::vector<MAP>& landmarks) {




    for(auto& l: landmarks)
    {

        // use global coordinate for markers
        Z(0) = l.second.distance;
        Z(1) = l.second.orientation;

        FieldLocation m = l.first;
        double dx = (m.x - state(0));
        double dy = (m.y - state(1));

        double q = pow(dx,2) + pow(dy,2);
        ZHAT(0) = sqrt(q);
        ZHAT(1) = atan2(dy, dx) - state(2);

        H(0, 0) =  -(dx/sqrt(q));
        H(0, 1) = -(dy/sqrt(q));
        H(0, 2) = 0;
        H(1, 0) = dy/q;
        H(1, 1) = -dx/q;
        H(1, 2) = -1;

        Q(0, 0) = pow(DETECTION_RANGE_ALPHA, 2);
        Q(1, 1) = pow(DETECTION_ANGLE_SIGMA, 2);

        S = H*P*H.transpose() + Q;
        K = P*H.transpose()*S.inverse();
        state_new = state_new + K*(Z - ZHAT);
        P = (I - K*H)*P;
        state_new(2) = constrain_angle(state_new(2));

    }

    state = state_new;
    P0 = P;

}

void EKF::nonZeroAngularVelocity(double v, double w, double theta) {


    G(0, 2) = -(v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
    G(1, 2) = -(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);

    V(0, 0) = (-sin(theta) + sin(theta + w*dt))/w;
    V(1, 0) = ( cos(theta) - cos(theta + w*dt))/w;
    V(0, 1) =  v*(sin(theta) - sin(theta + w*dt))/(w*w) + v*cos(theta + w*dt)*dt/w;
    V(1, 1) = -v*(cos(theta) - cos(theta + w*dt))/(w*w) + v*sin(theta + w*dt)*dt/w;
    V(2, 0) = 0;
    V(2, 1) = dt;

    // Prediction
    state_new(0) = state(0) - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
    state_new(1) = state(1) + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
    state_new(2) = state(2) + w*dt;

    P = G*P0*G.transpose() + V*R*V.transpose();

}

void EKF::zeroAngularVelocity(double v, double w, double theta) {

    // Handle case when w ~ 0
    // Use L'Hopital rule with lim w -> 0

    G(0, 2) = -v*sin(theta)*dt;
    G(1, 2) =  v*cos(theta)*dt;

    V(0, 0) = cos(theta)*dt;
    V(1, 0) = sin(theta)*dt;
    V(0, 1) = -v*sin(theta)*dt*dt*0.5;
    V(1, 1) =  v*cos(theta)*dt*dt*0.5;
    V(2, 0) = 0;
    V(2, 1) = dt;

    state_new(0) = state(0) + v*cos(theta)*dt;
    state_new(1) = state(1) + v*sin(theta)*dt;
    state_new(2) = state(2);

    P = G*P0*G.transpose() + V*R*V.transpose();

}



