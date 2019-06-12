
#ifndef KALMAN_FILTERS_QUATERNION_BASED_EXTENDED_KALMAN_FILTER_H_
#define KALMAN_FILTERS_QUATERNION_BASED_EXTENDED_KALMAN_FILTER_H_
/**************ATTENTION****************/
/*
* Quaternions in Eigen: q = (q_vec q_const)
* But this filter is based on: q = (q_const, q_vec)
* Consequently, if measurment provides quaternions: convert them first!!!
*/

#include <math.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

namespace KalmanFilters
{
class QuaternionBasedEKF {
public:

	/** 
	* Empty default constructor
	*/
	QuaternionBasedEKF();
	/**
	Constructor for Extended Kalman filter with the following parameters:
	*	dt 		  - time step
	*   q  		  - union quaternion for object orientation
	*	omega 	  - angular velocity
	* 	omega_dot - angular acceleration
	*	C  		  - Output martrix (assumption: constant matrix)
	*	Q  		  - process noise covariance matrix
	*	R 		  - measurement noise covariance matrix
	*	P  		  - error covariance matrix
	*/ 
	QuaternionBasedEKF(
		double dt,
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P
		);

	/** 
	* Initialize filter with initial states as zero
	*/
	void init();

	/** 
	* Initialize filter with a guess for initial states
	*/
	void init(double t0, const Eigen::VectorXd& x0);


	/**
	* Update the estimated state based on measured values
	* time step is assumed to remain constant
	*/
	void update(const Eigen::VectorXd& y);

	/** 
	* update estimated state based on measured values
	* using the given time step, nonlinear systems dynamics function and Jacobian of the nonlinear systems dynamic function
	*/ 
	void update(const Eigen::VectorXd& y, double dt, const Eigen::VectorXd f, const Eigen::MatrixXd F);

	/**
	* return: 
	*  - nonlinear dynamics function at current time step
	*  - linearized dynamics matrix at current time step
	*  - output matrix
	*  - process noise covariance matrix
	*  - measurement noise covariance matrix
	*  - error covariance matrix
	*  - current Kalman gain
	*  - current state 
	*  - time
	*/
	Eigen::VectorXd nonlinear_sys_dyn();
	Eigen::MatrixXd linearized_sys_dyn();
	Eigen::MatrixXd output();
	Eigen::MatrixXd process_noise();
	Eigen::MatrixXd measurement_noise();
	Eigen::MatrixXd error_cov();
	Eigen::MatrixXd kalman_gain();
	Eigen::VectorXd state();
	Eigen::Vector3d print_increment_func();
	double time();
	
private:

	/**
	* Computes the 2 Norm (Eucledian Norm)
	*/
	double TwoNorm(const Eigen::Vector3d& z); 

	/**
	* Computes the angular increment of a time step
	*/
	Eigen::Vector3d AngularIncrement(const Eigen::Vector3d w, const Eigen::Vector3d w_dot);

	/**
	* State evolution: computes f based on:
	* - exponential integration of q * e^(1/2*(omega*dt + 1/2*omega_dot*dt^2))
	* - Euler integration of omega
	* - Euler integration of omega_dot
	* evaluated at current state estimates
	*/
	void StateEvolution(); 

	/**
	* Computes the jacobian matrix F = df/dx evaluated at current state estimates
	*/
	void Jacobian();


	//nonlinear systems dynamics function
	Eigen::VectorXd f;

	//Matrices for computation - with Kalman gain K and initial error covariance matrix P0
	Eigen::MatrixXd F, C, Q, R, _P, K, P0;


	//System dimensions
	int m, n;

	//Initial and current time
	double t0, t;

	//discrete time step
	double dt;

	//Is the filter initialized?
	bool initialized;

	//n-size idenity
	Eigen::MatrixXd I;

	//Estimated states
	Eigen::VectorXd x_hat, x_hat_new;
	Eigen::VectorXd quat_hat;
	Eigen::Vector3d omega_hat;
	Eigen::Vector3d omega_dot_hat;

	//For debugging
	Eigen::Vector3d print_increment;




};
} /* namespace KalmanFilters */


#endif //KALMAN_FILTERS_EXTENDED_KALMAN_FILTER_H_
