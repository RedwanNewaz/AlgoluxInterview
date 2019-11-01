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

//    G.setIdentity();
//    V.setZero();
//    R.setZero();

    // convert delta to control inputs which are v (linear velocity) and omega (angular velocity)

    double v = sqrt(dX(0)*dX(0) + dX(1)*dX(1));
    double w = dX(2);
    double theta = state(2);
//
//    // noise
    R(0, 0) = pow(ALPHA1*fabs(v) + ALPHA2*fabs(w), 2);
    R(1, 1) = pow(ALPHA3*fabs(v) + ALPHA4*fabs(w), 2);


//    P.setIdentity();

    if(fabs(w) > EPS)
        nonZeroAngularVelocity(v,w,theta);
    else
        zeroAngularVelocity(v,w,theta);

    if(last_seen_landmarks_.size()>0)
        landmark_update(last_seen_landmarks_);



}

void EKF::landmark_update(const std::vector<MarkerObservation>& landmarks) {

//    Eigen::VectorXf ZHAT(2);
//    Eigen::MatrixXf H(2,3);
//
//    Q.setZero();
//    H.setZero();
//    I.setIdentity();

    /**
     * need to keep track of landmarks.
     * Canonically this function is called when landmarks are detected.
     * I also called this function when motion model is updated
     */
    if(last_seen_landmarks_.size()>0)
        for (int i = 0; i < last_seen_landmarks_.size(); ++i) {
            last_seen_landmarks_[i] = landmarks[i];
        }
    else
    std::copy(landmarks.begin(),landmarks.end(),std::back_inserter(last_seen_landmarks_));


    for (const auto &l : landmarks) {
        Z(0) = l.distance;
        Z(1) = l.orientation;

        if (fabs(l.distance) > EPS) {
            double range, bearing;

            ZHAT(0) = range;
            ZHAT(1) = bearing;

            ZHAT << l.distance, l.orientation;
            int index = l.markerIndex;

            H(0, 0) = -(true_landmarks_[index].x - state(0))/ZHAT(0);
            H(0, 1) = -(true_landmarks_[index].y - state(1))/ZHAT(0);
            H(0, 2) = 0;
            H(1, 0) = (true_landmarks_[index].y - state(1))/pow(ZHAT(0),2);
            H(1, 1) = -(true_landmarks_[index].x - state(0))/pow(ZHAT(0),2);
            H(1, 2) = -1;

            Q(0, 0) = pow(l.distance*DETECTION_RANGE_ALPHA, 2);
            Q(1, 1) = pow(l.orientation*DETECTION_ANGLE_SIGMA, 2);



            S = H*P*H.transpose() + Q;

            K = P*H.transpose()*S.inverse();
            state_new = state_new + K*(Z - ZHAT);
            P = (I - K*H)*P;

            state_new(2) = constrain_angle(state_new(2));

        }
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

double EKF::constrain_angle(double radian) {
    if (radian < -M_PI) {
        radian += 2*M_PI;
    } else if (radian > M_PI) {
        radian -= 2*M_PI;
    }

    return radian;
}


