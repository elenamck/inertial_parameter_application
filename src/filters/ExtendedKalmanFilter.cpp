#include <iostream>
#include <stdexcept>

#include "ExtendedKalmanFilter.h"

namespace KalmanFilters
{

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

ExtendedKalmanFilter::ExtendedKalmanFilter(
	double dt,
	const Eigen::VectorXd& f,
	const Eigen::MatrixXd& F,
	const Eigen::MatrixXd& C,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P)
: f(f), F(F), Q(Q), R(R), P0(P),
  m(C.rows()), n(F.rows()), dt(dt), initialized(false),
  I(n,n), x_hat(n), x_hat_new(n)

 {
 	I.setIdentity();
 }

void ExtendedKalmanFilter::init() {
	x_hat.setZero();
	P = P0; //Try P0 = I
	t0 = 0;
	t = t0;
	initialized = true;
}

void ExtendedKalmanFilter::init(double t0, const Eigen::VectorXd& x0){
	x_hat = x0;
	P = P0; //Try P0 = I
	_t0 = t0;
	t = t0;
	initialized = true;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& y){

	if(!initialized)
 		throw std::runtime_error("Extended Kalman Filter is not initialized!");
 	//First phase: Prediction
 	x_hat_new = f; //assuming no noise
 	P = F*P*F.transpose() + Q;

 	//Second phase: Correction of predicted variables based on Kalman gain
 	K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); //Kalman gain 
 	x_hat_new += K * (y - C * x_hat_new);
 	P = (I - K*C)*P;
 	x_hat = x_hat_new;

 	t+=dt;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& y, double dt, 
							      const Eigen::VectorXd f, const Eigen::MatrixXd F){
	_f = f;
	_F = F;
	_dt = dt;
	update(y);
}


Eigen::VectorXd ExtendedKalmanFilter::state() 
	{
		return x_hat;
	}

double ExtendedKalmanFilter::time()
{
	return t;
}
} /* namespace KalmanFilters */
