#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <math.h>
#include <stdexcept>
#include <iostream>
#include <eigen3/Eigen/Dense>

class KalmanFilter {
public: 

	/** 
	* Empty default constructor
	*/
	KalmanFilter();

	/**
	Constructor Kalman filter with the following matrices:
	* 	A - dynamics matrix
	*	C - output matrix
	*	Q - process noise covariance matrix
	*	R - measurement noise covariance matrix
	*	P - error covariance matrix
	*/
	KalmanFilter(
		double dt,
		const Eigen::MatrixXd& A,
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
	* using the given time step and dynamics matrix
	*/ 
	void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

	/**
	* return current state and time
	*/
	Eigen::VectorXd state();
	
	double time();

private:

	//Matrices for computation - with Kalman gain K and initial error covariance matrix P0
	Eigen::MatrixXd A, C, Q, R, P, K, P0;

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

#endif //KALMAN_FILTER_H_