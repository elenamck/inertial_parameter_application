#include "RecursiveLeastSquare.h"

#include <iostream>
#include <stdexcept>


namespace ParameterEstimation
{
RecursiveLeastSquare::RecursiveLeastSquare(int filter_size, const Eigen::MatrixXd& Lambda)
{	
	_accel_local.setZero();
	_aaccel_local.setZero();
	_avel_local.setZero();
	_g_local.setZero();
	_A_curr.setZero(6,10);
	_A.setZero(6,10); 
	_phi.setZero(10);
	_ft.setZero(6); 
	_FT.setZero(6); 
	_ft_contact.setZero(6); 
	_n_measurements = 0;
	_filter_size = filter_size;
	_Sigma.setZero(10,10);
	_K.setZero(10,6*_filter_size);
	_Lambda = Lambda;
	_Lambda_filt.setZero(6,6);
}

void RecursiveLeastSquare::init()
{
	_accel_local.setZero();
	_aaccel_local.setZero();
	_avel_local.setZero();
	_g_local.setZero();
	_A_curr.setZero(6,10);
	_A.setZero(6,10); 
	_phi.setZero(10);
	_ft.setZero(6); 
	_FT.setZero(6); 
	_ft_contact.setZero(6); 
	_n_measurements = 0;
	_Sigma.setZero(10,10);
	_K.setZero(10,6*_filter_size);
	_Lambda_filt.setZero(6,6);
}


void RecursiveLeastSquare::addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local)
{
	_accel_local  = accel_local;
	_avel_local   = avel_local;
	_aaccel_local = aaccel_local;
	_g_local 	  = g_local;


	getDataMatrix(_A_curr); 
	_ft = force_measurment;


	updateParameters();
	updateData();
	
}


void RecursiveLeastSquare::updateData()
{	
	_n_measurements++;

	if(_n_measurements==1)
	{
		_A = _A_curr;
		_FT = _ft;
	}
	else if (_n_measurements <= _filter_size)
	{
		Eigen::MatrixXd A_temp = _A;
		Eigen::VectorXd FT_temp = _FT;

		_A.resize(_n_measurements*6, 10);
		_FT.resize(_n_measurements*6);

		_A.bottomRows((_n_measurements-1)*6) = A_temp;
		_FT.bottomRows((_n_measurements-1)*6) = FT_temp;

		_A.topRows(6) = _A_curr;
		_FT.topRows(6) = _ft;

	}
	else if (_n_measurements > _filter_size)
	{
		Eigen::MatrixXd A_temp = _A.topRows((_filter_size-1)*6);
		Eigen::VectorXd FT_temp = _FT.topRows((_filter_size-1)*6);


		_A.bottomRows((_filter_size-1)*6) = A_temp;
		_FT.bottomRows((_filter_size-1)*6) = FT_temp;

		_A.topRows(6) = _A_curr;
		_FT.topRows(6) = _ft; 


	}
	

}





void RecursiveLeastSquare::updateParameters()
{

	
	if (_n_measurements == 0)
	{
		_Sigma = Eigen::MatrixXd::Identity(10,10);
		_Sigma *= 100;

		_Lambda_filt.resize(6*_filter_size, 6*_filter_size);

		for (int i=0; i<_filter_size; i++)
		{
			_Lambda_filt.block(i*6,i*6,6,6) = _Lambda;
		}

	}
	else if (_n_measurements >=_filter_size)
	{	

		_K = _Sigma*_A.transpose()*(_A*_Sigma*_A.transpose()+ _Lambda_filt).inverse();

		_Sigma = (Eigen::MatrixXd::Identity(10,10) - _K*_A)*_Sigma;
		_phi = _phi + _K*(_FT - _A*_phi);
	}
	


}



void RecursiveLeastSquare::getDataMatrix(Eigen::MatrixXd& A_data)
{
		A_data = Eigen::MatrixXd::Zero(6,10);
		for (int i=0; i<3; i++)    
			{
				for (int j=4; j<10; j++)
				{
					A_data(i,j)= 0.0;
				}	
			}
		for (int i=3; i<6; i++)
			{
				A_data(i,0)=0.0;
			} 	  
		A_data(3,1) = 0.0;
		A_data(4,2) = 0.0;
		A_data(5,3) = 0.0;	

		for (int i=0; i<3; i++)
			{
				A_data(i,0) = _accel_local(i)-_g_local(i);
			}

		A_data(0,1) = - _avel_local(1)*_avel_local(1) - _avel_local(2)*_avel_local(2);
		A_data(0,2) = _avel_local(0)*_avel_local(1) - _aaccel_local(2);
		A_data(0,3) = _avel_local(0)*_avel_local(2) + _aaccel_local(1);

		A_data(1,1) = _avel_local(0)*_avel_local(1) + _aaccel_local(2);
		A_data(1,2) = - _avel_local(0)*_avel_local(0) - _avel_local(2)*_avel_local(2);  
		A_data(1,3) = _avel_local(1)*_avel_local(2) - _aaccel_local(0);

		A_data(2,1) = _avel_local(0)*_avel_local(2) - _aaccel_local(1);
		A_data(2,2) = _avel_local(1)*_avel_local(2) + _aaccel_local(0);
		A_data(2,3) = - _avel_local(1)*_avel_local(1)-_avel_local(0)*_avel_local(0);

		A_data(3,2) = _accel_local(2) - _g_local(2);  
		A_data(3,3) = _g_local(1) - _accel_local(1);
		A_data(3,4) = _aaccel_local(0);
		A_data(3,5) = _aaccel_local(1) - _avel_local(0)*_avel_local(2);
		A_data(3,6) = _aaccel_local(2) + _avel_local(0)*_avel_local(1);
		A_data(3,7) = - _avel_local(1)*_avel_local(2);
		A_data(3,8) = _avel_local(1)*_avel_local(1) - _avel_local(2)*_avel_local(2);
		A_data(3,9) = _avel_local(1)*_avel_local(2);

		A_data(4,1) = _g_local(2) - _accel_local(2);
		A_data(4,3) = _accel_local(0) - _g_local(0);
		A_data(4,4) = _avel_local(0)*_avel_local(2);
		A_data(4,5) = _aaccel_local(0) + _avel_local(1)*_avel_local(2);
		A_data(4,6) = _avel_local(2)*_avel_local(2) - _avel_local(0)*_avel_local(0);
		A_data(4,7) = _aaccel_local(1);
		A_data(4,8) = _aaccel_local(2) - _avel_local(0)*_avel_local(1);
		A_data(4,9) = - _avel_local(0)*_avel_local(2);

		A_data(5,1) = _accel_local(1) - _g_local(1);
		A_data(5,2) = _g_local(0) - _accel_local(0);
		A_data(5,4) = - _avel_local(0)* _avel_local(1);
		A_data(5,5) = _avel_local(0)* _avel_local(0) - _avel_local(1)* _avel_local(1);
		A_data(5,6) = _aaccel_local(0) - _avel_local(1)*_avel_local(2);
		A_data(5,7) = _avel_local(0)*_avel_local(1);
		A_data(5,8) = _aaccel_local(1) + _avel_local(0)*_avel_local(2);	
		A_data(5,9) = _aaccel_local(2);	

}



Eigen::VectorXd RecursiveLeastSquare::computeContactForceTorque(const Eigen::VectorXd& force_torque_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local)
{
	_accel_local  = accel_local;
	_avel_local   = avel_local;
	_aaccel_local = aaccel_local;
	_g_local 	  = g_local;
	_phi     	  = phi;

	getDataMatrix(_A_curr);

	_ft_contact = force_torque_measured - _A_curr * _phi;

	return _ft_contact;
}

Eigen::VectorXd RecursiveLeastSquare::getInertialParameterVector()
{
	return _phi;	
}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentDataMatrixStacked()
{
	return _A;
}
Eigen::VectorXd RecursiveLeastSquare::getCurrentInputVectorStacked()
{

	return _FT;

}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentDataMatrix()
{
	return _A_curr;
}
Eigen::VectorXd RecursiveLeastSquare::getCurrentInputVector()
{

	return _ft;

}


Eigen::MatrixXd RecursiveLeastSquare::getCurrentGainMatrix()
{

		return _K;
}


Eigen::MatrixXd RecursiveLeastSquare::getCurrentParameterCovarianceMatrix()
{

	return _Sigma;
	

}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentNoiseCovarianceMatrix()
{

		return _Lambda;

}

}/* namespace ParameterEstimation */








