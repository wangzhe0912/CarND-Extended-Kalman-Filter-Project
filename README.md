# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project I will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

[//]: # (Image References)

[result]: ./img/result.jpg "result"


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Project Files

1. main.cpp: main.cpp is made up of several functions within main(), these all handle the uWebsocketIO communication between the simulator and it's self.
2. FusionEKF.cpp: In this file, you won't need to include the actual Kalman filter equations; instead, we will be initializing variables, initializing the Kalman filters, and then calling functions that implement the prediction step or update step.
3. kalman_filter.cpp: Implement the prediction and update equations for lidar and radar.
4. tools.cpp: This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix.

## Project Results
![alt text][image1]
