# Algolux Robot Localization Challenge

## Problem Statement 

The robot state is represented by a three-dimensional vector indicating the robot’s position and
orientation on the playing field: [x, y, θ]T. The goal is to implement a localization engine in
C++ that estimates the three-dimensional state of the robot at each time step.

##  Solution Approach
My main focus is to design an Extended Kalman Filter based localization engine because the robot might have limited computational resources. For the demonstration purpose of class inheritance,   
I have implemented two types of localization engines as follows
1. Extended Kalman Filter based engine (EKF engine),
2. Particle Filter based engine (PF engine).

However, since the problem requires to implement only one engine, 
I have entirely implemented the EKF engine, including the Kidnapping problem. 
If the robot gets kidnapped, it needs to look at a known landmark to relocalize itself. 
Due to time limitations, I haven't finished all the components of the particle filter based engine.  
*At the current stage, the PF engine can localize the robot successfully but cannot solve the kidnapping problem*. 
## Build
This project depends on following external libraries
* OpenGL
* GLUT
* Eigen3 3.3

I have used Eigen for Matrix multiplication. Since Eigen is very popular C++ library, I haven't included it with this project.
User needs to install it first to build this project.
  
### How to compile 
I assume that the user has Linux environment. 
This is a cmake project which can be compiled as follows
```asm
mkdir build && cd build 
cmake ..
```
Test inputs are located inside input folder. If you are in build directory, one may test a sample input file as follows 
```asm
./localization ../input/smaple_input1.txt
``` 

### Folder and Class description
I have created five folders to organize this project. 

The directory structure of this repository is as follows:

```
root
|   build
|   
|___cmake
|   |   FindController.cmake
|___engine
|   |   FilterBase.cpp
|   |   EKF.cpp
|   |   ParticleFilter.cpp
|___include
|   |   FilterBase.cpp
|   |   controller.h
|   |   EKF.h
|   |   FilterBase.h
|   |   main.h
|   |   ParticleFilter.h
|   |   robot_defs.h
|___input
|   |   sample_input1.txt
|   |   sample_input2.txt
|   |   sample_input3.txt
|___lib
|   |   libController.a
|   
|___main.cpp
```

As I have mentioned earlier, there are two engines that we can use for localization purposes. 
There are two main classes - EKF, ParticleFilter. Both of these classes are inherited from FilterBase class.
FilterBase class is an abstract class where I have implemented the common functions and variables used by derived classes.

### Coding style
Each header file and the source file is **Heavily-Commented**. I have written my thought process while coding.
I have followed the math symbols from the robotics book mostly. I have used underscore to denote member variables except for *state and state_new*.

### Switching Filters
I have defined KALMAN in __main.cpp__. This will enable compiler to build the EKF engine. 
Uncomment the definition KALMAN at __main.cpp__ file if you want to see the PF engine. 

## Future Work
For some unknown reasons, the OpenGL window is tiny on my screen.
There is also an optional keyboard input handling procedures that I haven't implemented yet. I found that mouse inputs are enough for my testing purposes. There are two libraries I wish I could use for this project 
1. Google test - for unit testing 
2. Google benchmark - for benchmarking the performance of different filters.   
