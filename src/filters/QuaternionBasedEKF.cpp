#include <iostream>
#include <stdexcept>
#include <math.h>

#include "QuaternionBasedEKF.h"

namespace KalmanFilters
{

QuaternionBasedEKF::QuaternionBasedEKF() {}

QuaternionBasedEKF::QuaternionBasedEKF(
	double dt,
	const Eigen::MatrixXd& C,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P)
: C(C), Q(Q), R(R), P0(P),
  m(C.rows()), n(10), dt(dt), initialized(false),
  I(n,n), x_hat(n), x_hat_new(n)

 {
 	I.setIdentity();
 	f = Eigen::VectorXd::Zero(n);
 	F = Eigen::MatrixXd::Zero(n,n);
 	K = Eigen::MatrixXd::Zero(10,n);
 	_P = Eigen::MatrixXd::Zero(10,10);
 	quat_hat = Eigen::VectorXd::Zero(4) ;
	omega_hat = Eigen::Vector3d::Zero();
	omega_dot_hat = Eigen::Vector3d::Zero();
	print_increment = Eigen::Vector3d::Zero();
 }


void QuaternionBasedEKF::init() {
	x_hat.setZero();
	_P = P0; //Try P0 = I
	t0 = 0;
	t = t0;
	initialized = true;
}

void QuaternionBasedEKF::init(double t0, const Eigen::VectorXd& x0){
	x_hat = x0;
	_P = P0; //Try P0 = I
	this->t0 = t0;
	t = t0;
	initialized = true;
}


double QuaternionBasedEKF::TwoNorm(const Eigen::Vector3d& z){

	return (sqrt(z(0)*z(0)+z(1)*z(1)+z(2)*z(2)));

}

Eigen::Vector3d QuaternionBasedEKF::AngularIncrement(const Eigen::Vector3d w, const Eigen::Vector3d w_dot){

	return (0.5*(w*dt + 0.5*w_dot*dt*dt));
}


void QuaternionBasedEKF::StateEvolution(){
	quat_hat << x_hat(0), x_hat(1), x_hat(2), x_hat(3); 
	omega_hat << x_hat(4), x_hat(5), x_hat(6);
	omega_dot_hat << x_hat(7), x_hat(8), x_hat(9);
	Eigen::Vector3d increment = AngularIncrement(omega_hat, omega_dot_hat); 
	double increment_norm = TwoNorm(increment);
	double c = cos(increment_norm);
	double s = sin(increment_norm);
	double n1, n2, n3; //normalized Angular increments 

	if(increment_norm == 0) 
	{
		n1 = 0;
		n2 = 0; 
		n3 = 0; 
	}

	else
	{
		n1 = increment(0)/increment_norm;
		n2 = increment(1)/increment_norm;
		n3 = increment(2)/increment_norm;
	}

	f(0) = quat_hat(0) * c + s * (-quat_hat(1) * n1 - quat_hat(2) * n2 - quat_hat(3) * n3);
	f(1) = quat_hat(1) * c + s * ( quat_hat(0) * n1 + quat_hat(3) * n2 - quat_hat(2) * n3);
	f(2) = quat_hat(2) * c + s * (-quat_hat(3) * n1 + quat_hat(0) * n2 + quat_hat(1) * n3);
	f(3) = quat_hat(3) * c + s * ( quat_hat(2) * n1 - quat_hat(1) * n2 + quat_hat(0) * n3);
	f(4) = omega_hat(0) + omega_dot_hat(0)*dt;
	f(5) = omega_hat(1) + omega_dot_hat(1)*dt;
	f(6) = omega_hat(2) + omega_dot_hat(2)*dt;
	f(7) = omega_dot_hat(0);
	f(8) = omega_dot_hat(1);
	f(9) = omega_dot_hat(2);
}

void QuaternionBasedEKF::Jacobian(){

	Eigen::Vector3d increment = AngularIncrement(omega_hat, omega_dot_hat); 
	print_increment = increment;
	Eigen::Vector3d increment_aux = 2.0 * increment; //for facilitating the implementation of partial derivatives
	double increment_norm = TwoNorm(increment);
	double increment_norm_squared = pow(increment_norm,2.0);
	double c = cos(increment_norm);
	double s = sin(increment_norm);
	double n1, n2, n3;

	Eigen::Vector3d p_diff_w_norm_incr = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_cos = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_sin = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_norm_incr = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_cos = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_sin = Eigen::Vector3d::Zero();

	Eigen::Vector3d p_diff_w_norm_incr_mixed = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_norm_incr_mixed = Eigen::Vector3d::Zero();


	if(increment_norm == 0)  //make sure not to divide through
	{
		n1 = 0;
		n2 = 0; 
		n3 = 0; 
	}

	else
	{
		n1 = increment(0)/increment_norm;
		n2 = increment(1)/increment_norm;
		n3 = increment(2)/increment_norm;
		//0 -> d (increment_y/norm(increment)) / d omega_x = d (increment_x/norm(increment)) / d omega_y
		//1 -> d (increment_z/norm(increment)) / d omega_x = d (increment_x/norm(increment)) / d omega_z
		//2 -> d (increment_z/norm(increment)) / d omega_y = d (increment_y/norm(increment)) / d omega_z
		p_diff_w_norm_incr_mixed(0) = - (dt * increment_aux(0) * increment_aux(1))/ (8.0 * pow(increment_norm_squared, 1.5));
		p_diff_w_norm_incr_mixed(1) = - (dt * increment_aux(0) * increment_aux(2))/ (8.0 * pow(increment_norm_squared, 1.5));
		p_diff_w_norm_incr_mixed(2) = - (dt * increment_aux(1) * increment_aux(2))/ (8.0 * pow(increment_norm_squared, 1.5));



		
		//0 -> d (increment_y/norm(increment)) / d omega_dot_x = d (increment_x/norm(increment)) / d omega_y_dot
		//1 -> d (increment_z/norm(increment)) / d omega_dot_x = d (increment_x/norm(increment)) / d omega_z_dot
		//2 -> d (increment_z/norm(increment)) / d omega_dot_y = d (increment_y/norm(increment)) / d omega_z_dot
		p_diff_w_dot_norm_incr_mixed(0) = -(pow(dt,2.0) * increment_aux(0) * increment_aux(1)) / (16.0 * pow(increment_norm_squared,1.5));
		p_diff_w_dot_norm_incr_mixed(1) = -(pow(dt,2.0) * increment_aux(0) * increment_aux(2)) / (16.0 * pow(increment_norm_squared,1.5));
		p_diff_w_dot_norm_incr_mixed(2) = -(pow(dt,2.0) * increment_aux(1) * increment_aux(2)) / (16.0 * pow(increment_norm_squared,1.5));


		/**computes partial derivatives 
		* - d(increment_i/norm(increment))/ d omega_i
		* - d(increment_i/norm(increment))/ d omega_dot_i
		* - d(cos(norm(increment)))/ d omega_i
		* - d(cos(norm(increment)))/ d omega_dot_i
		* - d(sin(norm(increment)))/ d omega_i
		* - d(sin(norm(increment)))/ d omega_dot_i
		*/
		for(int i=0; i<3; i++)
		{
			p_diff_w_norm_incr(i)     = dt/(2.0*increment_norm) - dt * pow(increment_aux(i),2.0)/(8.0*pow(increment_norm_squared,1.5));
			p_diff_w_dot_norm_incr(i) = pow(dt,2.0)/(4.0*increment_norm) - pow(dt,2.0) * pow(increment_aux(i),2.0)/(16.0*pow(increment_norm_squared,1.5));

			p_diff_w_cos(i)     = - (dt * increment_aux(i) * sin(increment_norm))/(4.0*increment_norm);
			p_diff_w_dot_cos(i) = - (pow(dt,2.0) * increment_aux(i) * sin(increment_norm))/(8.0*increment_norm);

			p_diff_w_sin(i)     =   (dt * increment_aux(i) * cos(increment_norm))/(4.0*increment_norm);
			p_diff_w_dot_sin(i) =   (pow(dt,2.0) * increment_aux(i) * cos(increment_norm))/(8.0*increment_norm);

		}

	}



	Eigen::MatrixXd quat_hat_coeffs = Eigen::MatrixXd::Zero(4,4);
	quat_hat_coeffs <<   quat_hat(0),   quat_hat(1),   quat_hat(2),   quat_hat(3),
					   - quat_hat(1),   quat_hat(0), - quat_hat(3),   quat_hat(2),
					   - quat_hat(2),   quat_hat(3),   quat_hat(0), - quat_hat(1),
					   - quat_hat(3), - quat_hat(2),   quat_hat(1),   quat_hat(0);


	//Partial derivatives of the quaternions
	F.block(0,0,1,4) <<      c, -n1 * s, -n2 * s, -n3 * s;
	F.block(1,0,1,4) << n1 * s,       c, -n3 * s,  n2 * s;
	F.block(2,0,1,4) << n2 * s,  n3 * s,       c, -n1 * s;
	F.block(3,0,1,4) << n3 * s, -n2 * s, -n1 * s,       c;

	//partial derivatives of angular velocity and angular acceleration
	for(int i=0; i<4; i++)
	{

		F(i,4) = quat_hat_coeffs(0,i)*p_diff_w_cos(0) + p_diff_w_sin(0)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_norm_incr(0) + quat_hat_coeffs(2,i)*p_diff_w_norm_incr_mixed(0)
				 + quat_hat_coeffs(3,i)*p_diff_w_norm_incr_mixed(1));

		F(i,5) = quat_hat_coeffs(0,i)*p_diff_w_cos(1) + p_diff_w_sin(1)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_norm_incr_mixed(0) + quat_hat_coeffs(2,i)*p_diff_w_norm_incr(1)
				 + quat_hat_coeffs(3,i)*p_diff_w_norm_incr_mixed(2));

		F(i,6) = quat_hat_coeffs(0,i)*p_diff_w_cos(2) + p_diff_w_sin(2)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_norm_incr_mixed(1) + quat_hat_coeffs(2,i)*p_diff_w_norm_incr_mixed(2)
				 + quat_hat_coeffs(3,i)*p_diff_w_norm_incr(2));

		F(i,7) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(0) + p_diff_w_dot_sin(0)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_dot_norm_incr(0) + quat_hat_coeffs(2,i)*p_diff_w_dot_norm_incr_mixed(0)
				 + quat_hat_coeffs(3,i)*p_diff_w_dot_norm_incr_mixed(1));
				 
		F(i,8) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(1) + p_diff_w_dot_sin(1)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_dot_norm_incr_mixed(0) + quat_hat_coeffs(2,i)*p_diff_w_dot_norm_incr(1)
				 + quat_hat_coeffs(3,i)*p_diff_w_dot_norm_incr_mixed(2));

		F(i,9) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(2) + p_diff_w_dot_sin(2)*( quat_hat_coeffs(1,i) * n1 + quat_hat_coeffs(2,i) * n2 + quat_hat_coeffs(3,i) * n3) 
				 + s * (quat_hat_coeffs(1,i)*p_diff_w_dot_norm_incr_mixed(1) + quat_hat_coeffs(2,i)*p_diff_w_dot_norm_incr_mixed(2)
				 + quat_hat_coeffs(3,i)*p_diff_w_dot_norm_incr(2));
	}
	//Partial derivatives of Euler Integration part
	F.block(4,4,3,3) = Eigen::Matrix3d::Identity();
	F.block(4,7,3,3) = dt*Eigen::Matrix3d::Identity();
	F.block(7,7,3,3) = Eigen::Matrix3d::Identity();
}

void QuaternionBasedEKF::update(const Eigen::VectorXd& y){

	if(!initialized)
 		throw std::runtime_error("Extended Kalman Filter is not initialized!");
 	//First phase: Prediction
 	StateEvolution(); 
 	x_hat_new = f; //assuming no noise
 	Jacobian(); //compute F
 	_P = F*_P*F.transpose() + Q;

 	//Second phase: Correction of predicted variables based on Kalman gain
 	K = _P*C.transpose()*(C*_P*C.transpose() + R).inverse(); //Kalman gain 
 	x_hat_new += K * (y - C * x_hat_new);
 	_P = (I - K*C)*_P;
 	x_hat = x_hat_new;

 	t+=dt;
}




Eigen::VectorXd QuaternionBasedEKF::state() 
{
	return x_hat;
}

double QuaternionBasedEKF::time()
{
	return t;
}

Eigen::VectorXd QuaternionBasedEKF::nonlinear_sys_dyn()
{
	return f;
}

Eigen::MatrixXd QuaternionBasedEKF::linearized_sys_dyn()
{
	return F;
}

Eigen::MatrixXd QuaternionBasedEKF::output()
{
	return C;
}
	
Eigen::MatrixXd QuaternionBasedEKF::process_noise()
{
	return Q;
}

Eigen::MatrixXd QuaternionBasedEKF::measurement_noise()
{
	return R;
}

Eigen::MatrixXd QuaternionBasedEKF::error_cov()
{
	return _P;
}

Eigen::MatrixXd QuaternionBasedEKF::kalman_gain()
{
	return K;
}

Eigen::Vector3d QuaternionBasedEKF::print_increment_func()
{
	return print_increment;
}
} /* namespace KalmanFilters */


