#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"
#include "EKF.h"

EKF stateEstimator;

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
    // TODO: Write your procedures to set the current robot position estimate here
    auto X = stateEstimator.get_state();
    
    estimatePosn.x = X(0);
    estimatePosn.y = X(1);
    estimatePosn.theta = X(2);
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // TODO: Write your motion update procedures here
    Eigen::VectorXd dX(STATE_SIZE);
    dX << delta.x, delta.y, delta.theta;
    stateEstimator.predict(dX);

    
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    // TODO: Write your sensor update procedures here
    stateEstimator.landmark_update(observations);
    
}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // TODO: Write your initialization procedures here
    Eigen::VectorXd x0(STATE_SIZE);
    x0<<robotState.x,robotState.y,robotState.theta;
    std::vector<FieldLocation> landmarks;
    for(int i = 0; i<NUM_LANDMARKS; ++i)
        landmarks.emplace_back(markerLocations[i]);
    stateEstimator.set(robotParams, landmarks);
    stateEstimator.init(1, x0);
    
}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // TODO: Write your drawing procedures here 
    //       (e.g., robot position uncertainty representation)

    stateEstimator.render_ellipse();

}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here
	
	return 0;
}


/**
 * Main entrypoint for the program.
 */
int main (int argc, char ** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);

    return 0;
}

