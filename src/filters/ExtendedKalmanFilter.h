#ifndef KALMAN_FILTERS_EXTENDED_KALMAN_FILTER_H_
#define KALMAN_FILTERS_EXTENDED_KALMAN_FILTER_H_

#include <math.h>
#include <stdexcept>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace KalmanFilters
{

class ExtendedKalmanFilter {
public:

	/** 
	* Empty default constructor
	*/
	ExtendedKalmanFilter();
	/**
	Constructor for Extended Kalman filter with the following parameters:
	*	f - nonlinear systems dynamics function
	*	F - Jacobian of the nonlinear systems dynamics function, evaluated at the current state
	*	C - Output martrix (assumption: constant matrix)
	*	Q - process noise covariance matrix
	*	R - measurement noise covariance matrix
	*	P - error covariance matrix
	*/ 
	ExtendedKalmanFilter(
		double dt,
		const Eigen::VectorXd& f,
		const Eigen::MatrixXd& F,
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
	* return current state and time
	*/
	Eigen::VectorXd state();

	double time();
	
private:
	//nonlinear systems dynamics function
	Eigen::VectorXd f;

	//Matrices for computation - with Kalman gain K and initial error covariance matrix P0
	Eigen::MatrixXd F, C, Q, R, P, K, P0;


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



};
}  /* namespace KalmanFilters */

#endif //KALMAN_FILTERS_EXTENDED_KALMAN_FILTER_H_
