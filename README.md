# inertial_parameter_application
*Multisensor-based parameter estimation on FrankaEmika's Panda arm*
## Overview

* 01-panda_force_control:
 * 01-kalman_filters: controllers for testing and tuning of linear and extended kalman filters
 * 02-adaptive_filters: controllers with functions for estimation in main
 * 03-trajectories: controllers for testing different trajectories
 * 04-SVH_hand: controllers for grasping
 * 05-estimation_controllers: current testing files
* 02-utilities: 
 * includes controller for force sensor calibration and controller for checking wether the IMU frame is transformed correctly in the force sensor frame
* data_collection: plotting and logging python scripts for redis-keys
* robot_models: includes models for FrankaEmika Panda arm and Kuka iiwa arm
* src: source and header files for inertial parameter application
  * filters: 
   * kalman filter
   * quaternionbased extended kalman filter
   * savitzky golay filter *(part if rocalib)*
  * parameter_estimation: 
   * least square 
   * recursive least square


