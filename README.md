# inertial_parameter_application

## Overview

Tools for inertial parameter estimation on FrankaEmika Panda arm.

* 01-panda_force_control: controllers for testing 
* 02-utilities: includes controller for force sensor calibration and controller for checking wether the IMU frame is transformed correctly in the force sensor frame
* data_collection: plotting and logging scripts for redis-keys
* robot_models: includes models for FrankaEmika Panda arm and Kuka iiwa arm
* src: source and header files for inertial parameter application
  * filters: implementations Kalman Filter, Extended Kalman filter for kinematic variables
  * parameter_estimation: implementations of different estimation algorithms



