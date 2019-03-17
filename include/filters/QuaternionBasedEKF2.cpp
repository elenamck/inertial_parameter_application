#include <iostream>
#include <stdexcept>
#include <math.h>

#include "QuaternionBasedEKF2.h"

QuaternionBasedEKF2::QuaternionBasedEKF2() {}

QuaternionBasedEKF2::QuaternionBasedEKF2(
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
 	K = Eigen::MatrixXd::Zero(n,m);
 	_P = Eigen::MatrixXd::Zero(n,n);
 	quat_hat = Eigen::VectorXd::Zero(4) ;
	omega_hat = Eigen::Vector3d::Zero();
	omega_dot_hat = Eigen::Vector3d::Zero();
	print_increment = Eigen::Vector3d::Zero();
 }


void QuaternionBasedEKF2::init() {
	x_hat.setZero();
	_P = P0; //Try P0 = I
	t0 = 0;
	t = t0;
	initialized = true;
}

void QuaternionBasedEKF2::init(double t0, const Eigen::VectorXd& x0){
	x_hat = x0;
	_P = P0; //Try P0 = I
	this->t0 = t0;
	t = t0;
	initialized = true;
}


double QuaternionBasedEKF2::TwoNorm(const Eigen::Vector3d& z){

	return (sqrt(z(0)*z(0)+z(1)*z(1)+z(2)*z(2)));

}

Eigen::Vector3d QuaternionBasedEKF2::AngularIncrement(const Eigen::Vector3d w, const Eigen::Vector3d w_dot){

	return (0.5*(w*dt + 0.5*w_dot*dt*dt));
}


void QuaternionBasedEKF2::StateEvolution(){
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

void QuaternionBasedEKF2::Jacobian(){

	Eigen::Vector3d increment = AngularIncrement(omega_hat, omega_dot_hat); 
	print_increment = increment;
	double increment_norm = TwoNorm(increment);
	double increment_norm_reciprocal = 1.0 / increment_norm;
	double c = cos(increment_norm);
	double s = sin(increment_norm);
	double n1, n2, n3;

	Eigen::Vector3d p_diff_w_norm_reciprocal = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_cos = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_sin = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_norm_reciprocal = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_cos = Eigen::Vector3d::Zero();
	Eigen::Vector3d p_diff_w_dot_sin = Eigen::Vector3d::Zero();

	double p_diff_w_increment = 0.0;								//d(increment_i)/d(omega_i)      -> equal for all indexes
	double p_diff_w_dot_increment = 0.0;							//d(increment_i)/d(omega_dot_i)  -> equal for all indexes


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



		/**computes partial derivatives 
		* - d(1/norm(increment))/ d omega_i
		* - d(1/norm(increment))/ d omega_dot_i
		* - d(cos(norm(increment)))/ d omega_i
		* - d(cos(norm(increment)))/ d omega_dot_i
		* - d(sin(norm(increment)))/ d omega_i
		* - d(sin(norm(increment)))/ d omega_dot_i
		*/
		for(int i=0; i<3; i++)
		{
			p_diff_w_norm_reciprocal(i)     = - dt * increment(i) / (2.0* pow(increment_norm,3.0));																//checked
			p_diff_w_dot_norm_reciprocal(i) = - pow(dt,2.0) * increment(i) / (4.0 * pow(increment_norm,3.0));													//checked

			p_diff_w_cos(i)     = - (dt * increment(i) * s)/(2.0*increment_norm);																				//checked
			p_diff_w_dot_cos(i) = - (pow(dt,2.0) * increment(i) * s)/(4.0*increment_norm);																		//checked

			p_diff_w_sin(i)     =   (dt * increment(i) * c)/(2.0*increment_norm);
			p_diff_w_dot_sin(i) =   (pow(dt,2.0) * increment(i) * c)/(4.0*increment_norm);

		}

		p_diff_w_increment 	   = dt/2.0;
		p_diff_w_dot_increment = pow(dt,2.0) / 4.0;

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
	F.block(3,0,1,4) << n3 * s, -n2 * s,  n1 * s,       c;

	//partial derivatives of angular velocity and angular acceleration
	for(int i=0; i<4; i++)
	{

		F(i,4) = quat_hat_coeffs(0,i)*p_diff_w_cos(0) + p_diff_w_norm_reciprocal(0) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_sin(0) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(1,i) * s * p_diff_w_increment;


		F(i,5) = quat_hat_coeffs(0,i)*p_diff_w_cos(1) + p_diff_w_norm_reciprocal(1) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_sin(1) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(2,i) * s * p_diff_w_increment;

		F(i,6) = quat_hat_coeffs(0,i)*p_diff_w_cos(2) + p_diff_w_norm_reciprocal(2) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_sin(2) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(3,i) * s * p_diff_w_increment;

		F(i,7) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(0) + p_diff_w_dot_norm_reciprocal(0) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_dot_sin(0) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(1,i) * s * p_diff_w_dot_increment;
				 
		F(i,8) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(1) + p_diff_w_dot_norm_reciprocal(1) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_dot_sin(0) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(2,i) * s * p_diff_w_dot_increment;

		F(i,9) = quat_hat_coeffs(0,i)*p_diff_w_dot_cos(2) + p_diff_w_dot_norm_reciprocal(2) * s * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) +
				 increment_norm_reciprocal * p_diff_w_dot_sin(2) * (quat_hat_coeffs(1,i) * increment(0) + quat_hat_coeffs(2,i) * increment(1) + quat_hat_coeffs(3,i) * increment(2)) 
				 + quat_hat_coeffs(3,i) * s * p_diff_w_dot_increment;
	}
	//Partial derivatives of Euler Integration part
	F.block(4,4,3,3) = Eigen::Matrix3d::Identity();
	F.block(4,7,3,3) = dt*Eigen::Matrix3d::Identity();
	F.block(7,7,3,3) = Eigen::Matrix3d::Identity();
}

void QuaternionBasedEKF2::update(const Eigen::VectorXd& y){

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

void QuaternionBasedEKF2::update(const Eigen::VectorXd& y, double dt, 
							    const Eigen::VectorXd f, const Eigen::MatrixXd F){
	this->f = f;
	this->F = F;
	this->dt = dt;
	update(y);
}


Eigen::VectorXd QuaternionBasedEKF2::state() 
{
	return x_hat;
}

double QuaternionBasedEKF2::time()
{
	return t;
}

Eigen::VectorXd QuaternionBasedEKF2::nonlinear_sys_dyn()
{
	return f;
}

Eigen::MatrixXd QuaternionBasedEKF2::linearized_sys_dyn()
{
	return F;
}

Eigen::MatrixXd QuaternionBasedEKF2::output()
{
	return C;
}
	
Eigen::MatrixXd QuaternionBasedEKF2::process_noise()
{
	return Q;
}

Eigen::MatrixXd QuaternionBasedEKF2::measurement_noise()
{
	return R;
}

Eigen::MatrixXd QuaternionBasedEKF2::error_cov()
{
	return _P;
}

Eigen::MatrixXd QuaternionBasedEKF2::kalman_gain()
{
	return K;
}

Eigen::Vector3d QuaternionBasedEKF2::print_increment_func()
{
	return print_increment;
}


