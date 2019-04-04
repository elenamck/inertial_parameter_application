
#ifndef TRAJECTORIES_JOINT_SPACE_SINUSODIAL_H_
#define TRAJECTORIES_JOINT_SPACE_SINUSODIAL_H_

#include <Eigen/Dense>


namespace Trajectories
{
class JointSpaceSinusodial {
public: 

	/** 
	* Empty default constructor
	*/
	JointSpaceSinusodial();

	/**
	Constructor periodic joint trajectories based on fourier series:
	*	axis - Number of joint axis for which to compute trajectories
	*	N - Number of superposed trajectories
	*	w_s - sampling frequency
	*	w_f - fundamental pulsation of the Fourier series
	* 	a - coevicients of the fourier series a (if zero: output trajectory odd q(-x) = -q(x))
	* 	b - coevicients of the fourier series b (if zero: output trajectory even q(-x) = q(x))
	*/
	JointSpaceSinusodial(
		const int axis,
		const int N, 
		const double w_s,
		const double w_f,
		Eigen::VectorXd a,
		Eigen::VectorXd b);


	/**
	Constructor periodic joint trajectories predetermined coefficients on fourier series:
	*	N - Number of superposed trajectories
	*	w_s - sampling frequency
	*/
	JointSpaceSinusodial(
		const int N, 
		const double w_s);

	/**
	* @brief Inititializes trajectories
	* @param q_0 initial configuration, offset in joint angles
	*/
	void init(const Eigen::VectorXd& q_0);

	/**
	* @brief Inititializes trajectories
	* @param q_0 - initial configuration, offset in joint angles
	* @param a   - coevicients of the fourier series a (if zero: output trajectory odd q(-x) = -q(x))
	* @param b   - coevicients of the fourier series b (if zero: output trajectory even q(-x) = q(x))
	*/
	void init(const Eigen::VectorXd& q_0, Eigen::VectorXd a, Eigen::VectorXd b);

	/**
	* @brief Updates the trajectory
	* @param counter - current time step
	*/
	void update(int counter);

	/**
	* @brief returns current joint angles
	*/
	Eigen::VectorXd getJointAngles();

	/**
	* @brief returns current joint velocities
	*/
	Eigen::VectorXd getJointVelocities();

	/**
	* @brief returns current joint accelerations
	*/
	Eigen::VectorXd getJointAccelerations();



private:
	int _axis;				//Number of joint axis for which to compute trajectories
	int _N; 					//Number of superposed trajectories	
	int _k; 					//discrete time
	int _n;					//Number of fourier coeffiecients a (_n=_m, _n=_N*_axis)
	int _m;					//Number of fourier coeffiecients b (_m=_n, _m=_N*_axis)
	double _w_s;				//sampling frequency
	double _w_f;				//fundamental pulsation of the Fourier series
	double _T_s; 				//sampling time
	double _arg;				//argument of periodic functions
	double _arg_diff;			//differentiated argument of periodic functions
	double _arg_diff_diff;	//two times differentiated argument of periodic functions

	Eigen::VectorXd _a;				//coevicients of the fourier series a (if zero: output trajectory odd q(-x) = -q(x))
	Eigen::VectorXd _b;				//coevicients of the fourier series b (if zero: output trajectory even q(-x) = q(x))
	Eigen::VectorXd _q0;			//joint angle offset

	Eigen::MatrixXd _M_a;			//Matrix with coefficient a for each axis in each column
	Eigen::MatrixXd _M_b;			//Matrix with coefficient b for each axis in each column

	Eigen::VectorXd _q;				//resulting trajectory pos (joint_task->_goal_position.tail(X))
	Eigen::VectorXd _dq;			//resulting trajectory avel (joint_task->_desired_velocity.tail(X))
	Eigen::VectorXd _ddq;			//resulting trajectory aaccel 


};

} /* namespace Trajectories */


#endif //JOINT_SPACE_SINUSODIAL_H_