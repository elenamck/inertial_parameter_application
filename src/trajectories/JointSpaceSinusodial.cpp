#include "JointSpaceSinusodial.h"

#include <Eigen/Dense>
#include <iostream>




namespace Trajectories
{
JointSpaceSinusodial::JointSpaceSinusodial(
		const int axis,
		const int N, 
		const double w_s,
		const double w_f,
		Eigen::VectorXd a,
		Eigen::VectorXd b) 

{
	_axis = axis;
	_n = a.size();
	_m = b.size();
	_q.setZero(_axis);				//resulting trajectory pos (joint_task->_goal_position.tail(X))
	_dq.setZero(_axis);				//resulting trajectory avel (joint_task->_desired_velocity.tail(X))
	_ddq.setZero(_axis);			//resulting trajectory aaccel 
	_q0.setZero(_axis);
	_a.setZero(_n);
	_b.setZero(_m);	
	_a = a;
	_b = b;	
	_N = N;
	_w_s = w_s;
	_w_f = w_f;
	_T_s = 1/w_s;
	_arg = 0;
	_arg_diff = 0;
	_arg_diff_diff = 0;
	_k = 0;
	_M_a.setZero(_N, _axis);
	_M_b.setZero(_N, _axis);
}

void JointSpaceSinusodial::init(const Eigen::VectorXd& q_0)
{
	_q0 = q_0; 

}
void JointSpaceSinusodial::init(const Eigen::VectorXd& q_0, Eigen::VectorXd a, Eigen::VectorXd b)
{
	_q0 = q_0; 
	_a = a;
	_b = b;

}
void JointSpaceSinusodial::update(int counter)
{	
	_q.setZero(_axis);				
	_dq.setZero(_axis);				
	_ddq.setZero(_axis);
	if(_n != _m)
	{
		throw std::invalid_argument("Number of fourier coeffients a doesn't match number of fourier coeffients b!\n");
	}
	else if(_n != _axis*_N)
	{
		throw std::invalid_argument("Number of fourier coeffients doesn't match desired axis times number of superposed trajectories!\n");
	}
	for(int cols = 0; cols < _axis; cols++)
	{
		for(int rows = 0; rows < _N; rows++)
		{
			_M_a(rows, cols) = _a(rows+(cols*2));
			_M_b(rows, cols) = _b(rows+(cols*2));
		}
	}

	_k = counter;
	_arg = _w_f * _k * _T_s;
	_arg_diff = _w_f;
	_arg_diff_diff = _w_f*_w_f;


	for(int i = 0; i < _axis; i++)
	{
		_q(i) = _q0(i);
		for(int j=0; j < _N; j++)
		{
			_q(i) += _M_a(j,i) * sin(_arg*(j+1)) + _M_b(j,i) * cos(_arg*(j+1));
			_dq(i) += _M_a(j,i) * _arg_diff * (j+1) * cos(_arg*(j+1)) - _M_b(j,i) * _arg_diff * (j+1) * sin(_arg*(j+1));
			_ddq(i) += - _M_a(j,i) * _arg_diff_diff * (j+1)*(j+1) * sin(_arg*(j+1)) - _M_b(j,i) * _arg_diff_diff * (j+1)*(j+1) * cos(_arg*(j+1));
			//std::cout << "for i: " << i << "and for j: " << j << "a: " << _M_a(j,i) << "b: " << _M_b(j,i) << std::endl;
			//std::cout << "the desired joint angles are: " << _q.transpose()<< std::endl;
		} 
	}
	// std::cout << "for timestep: " << _k << " the desired joint angles are: " << _q.transpose()<< std::endl;
	// std::cout << "for timestep: " << _k << " the desired joint velcities are: " << _q.transpose()<< std::endl;
	// std::cout << "for timestep: " << _k << " the desired joint accelerations are: " << _q.transpose()<< std::endl;

}

Eigen::VectorXd JointSpaceSinusodial::getJointAngles()
{
	return _q;
}

Eigen::VectorXd JointSpaceSinusodial::getJointVelocities()
{
	return _dq;
}
Eigen::VectorXd JointSpaceSinusodial::getJointAccelerations()
{
	return _ddq;
}


}/* namespace Trajectories */