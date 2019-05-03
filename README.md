# inertial_parameter_application
*Multisensor-based parameter estimation on FrankaEmika's Panda arm*
## Overview

* 01-panda_force_control:
  * 01-kalman_filters: controllers for testing and tuning of linear and extended kalman filters
  * 02-adaptive_filters: controllers with functions for estimation in main
  * 03-trajectories: controllers for testing different trajectories
  * 04-SVH_hand: controllers for grasping
  * 05-estimation_controllers: current tests
  * 06-urdf_files: panda arm with/without virtual forcesensor, world files
  * 07-simulations: codes for simulation, sim file which sends linear acceleration, angular velocity/acceleration only runs at 2 kHz
* 02-utilities: 
  * includes controller for force sensor calibration and controller for checking wether the IMU frame is transformed correctly in the force sensor frame
* 03-tests:
  * test for mean filter
* 04-matlab:
  * matlab files for waypoint trajectory generation with inverse kinematics
* data_collection: plotting and logging python scripts for redis-keys
* robot_models: includes models for FrankaEmika Panda arm and Kuka iiwa arm
* src: source and header files for inertial parameter application
   * filters: 
     * kalman filter
     * quaternionbased extended kalman filter
     * savitzky golay filter *(part if rocalib, copyright CNRS-UM LIRMM)*
     * second order lowpass filter *(copyright Chair of Applied Mechanics, TUM)*
     * FIR filter *(copyright Chair of Applied Mechanics, TUM)*
  * parameter_estimation: 
    * least square 
    * recursive least square
  * trajectories:
    * fourierseries in python
    * sinusoidal joint space


