#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"

namespace KalmanFilters
{

KalmanFilter::KalmanFilter() {}

KalmanFilter::KalmanFilter(
	double dt,
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& C,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P)
: A(A), C(C), Q(Q), R(R), P0(P),
  m(C.rows()), n(A.rows()), dt(dt), initialized(false),
  I(n, n), x_hat(n), x_hat_new(n)

 {
 	I.setIdentity();
 }


 void KalmanFilter::init() {
 	x_hat.setZero();
 	P = P0; //Try P0 = I
 	t0 = 0;
 	t = t0;
 	initialized = true;
 }

 void KalmanFilter::init(double t0, const Eigen::VectorXd& x0){
 	x_hat = x0;
 	P = P0; //Try P0 = I
 	this->t0 = t0;
 	t = t0;
 	initialized = true;
 }

 void KalmanFilter::update(const Eigen::VectorXd& y){

 	if(!initialized)
 		throw std::runtime_error("Kalman Filter is not initialized!");
 	//First phase: Prediction
 	x_hat_new = A * x_hat; //assuming no noise
 	P = A*P*A.transpose() + Q;

 	//Second phase: Correction of predicted variables based on Kalman gain
 	K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); //Kalman gain
 	x_hat_new += K * (y - C*x_hat_new);
 	P = (I - K*C)*P;
 	x_hat = x_hat_new; 

 	t+=dt; 

 }


 Eigen::VectorXd KalmanFilter::state() 
	{
		return x_hat;
	}

double KalmanFilter::time()
{
	return t;
}
} /* namespace KalmanFilters */