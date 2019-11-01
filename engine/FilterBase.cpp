//
// Created by redwan on 10/31/19.
//

#include "FilterBase.h"


FilterBase::FilterBase() {

    // number of state variables (x,y,phi)
    n = STATE_SIZE;

    // Resizing matrices
    state.resize(n);
    state_new.resize(n);
    P0.resize(n, n); // Initial estimate error covariance

    dt = 1;

}

Eigen::VectorXd FilterBase::get_state() {

    return state_new;
}

void FilterBase::set(const RobotParams &parm, const std::vector<FieldLocation> &landmark) {

    std::copy(landmark.begin(),landmark.end(),std::back_inserter(true_landmarks_));


    DETECTION_RANGE_ALPHA = parm.sensor_noise_distance;
    DETECTION_ANGLE_SIGMA = parm.sensor_noise_orientation;

//    bad result if we initialize Q and R in here
//    Q(0, 0) = pow(parm.sensor_noise_distance, 2);
//    Q(1, 1) = pow(parm.sensor_noise_orientation, 2);

    ALPHA1 = parm.odom_noise_rotation_from_rotation;
    ALPHA2 = parm.odom_noise_rotation_from_translation;
    ALPHA3 = parm.odom_noise_translation_from_translation;
    ALPHA4 = parm.odom_noise_translation_from_rotation;

//      Don't do it - bad result
//    R(0, 0) = pow(parm.odom_noise_rotation_from_rotation + parm.odom_noise_rotation_from_translation, 2);
//    R(1, 1) = pow(ALPHA3 + ALPHA4, 2);


    // Don't forget to initialize specific filter later !
    initialized_ = false;
}

void FilterBase::render_ellipse() {
    double major, minor, theta;

//    Pxy = PEst[0:2, 0:2]

    Eigen::Matrix<double,2,2> Pxy;
    Pxy = P.block(0, 0, 2, 2);


    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> a(Pxy);
    double e0 = sqrt(a.eigenvalues()(0));
    double e1 = sqrt(a.eigenvalues()(1));

    printf("(%ld, %ld)\n", e0, e1);

    if (e0 > e1) {
        theta = atan2(a.eigenvectors()(1, 0), a.eigenvectors()(0, 0));
        major = e0;
        minor = e1;
    } else {
        theta = atan2(a.eigenvectors()(1, 1), a.eigenvectors()(0, 1));
        major = e1;
        minor = e0;
    }



    //    // Example drawing procedure
    int pixelX, pixelY, covX, covY;
    //// Draw cyan colored points at specified global locations on field
    glColor3f(0.0, 1.0, 1.0);

    global2pixel(state_new(0), state_new(1), pixelX, pixelY);

    glPushMatrix();
    glLoadIdentity();

    glTranslatef(pixelX, pixelY, 0);
    glRotatef(theta*180/M_PI, 0, 0, 1);

    // scale minor and major axises: 0.1 is a good number

    // draw major axis
    glBegin(GL_LINES);
    global2pixel(major, 0, covX, covY);
    major = covX*0.1;
    glVertex2f(-major, 0);
    glVertex2f(major, 0);
    glEnd();

    // draw minor axis
    glBegin(GL_LINES);
    global2pixel(0, minor, covX, covY);
    minor = covY*0.1;
    glVertex2f(0, -minor);
    glVertex2f(0, minor);
    glEnd();

    std::cout<<major<<"\t"<<minor<<"\n";

    // draw ellipise
    glBegin(GL_LINE_LOOP);
    for (float a=0; a < 360.0; a += 10.0) {
        double xe = major*cos(a*M_PI/180);
        double ye = minor*sin(a*M_PI/180);
        glVertex2f(xe, ye);
    }
    glEnd();

    glPopMatrix();

}

