#include "RecursiveLeastSquare.h"

#include <iostream>
#include <stdexcept>


namespace ParameterEstimation
{
RecursiveLeastSquare::RecursiveLeastSquare(bool lin, int filter_size, const Eigen::MatrixXd& Lambda)
{	
	_accel_local.setZero();
	_aaccel_local.setZero();
	_avel_local.setZero();
	_g_local.setZero();
	_A_curr.setZero(6,10);
	_A.setZero(6,10); 
	_A_lin_curr.setZero(3,4);
	_A_lin.setZero(3,4);
	_phi.setZero(10);
	_phi_lin.setZero(4); 
	_ft.setZero(6); 
	_FT.setZero(6); 
	_f.setZero();
	_F.setZero(3); 
	_ft_contact.setZero(6); 
	_f_contact.setZero(); 
	_n_measurements = 0;
	_linear_case = lin;
	_filter_size = filter_size;
	_Sigma.setZero(10,10);
	_Sigma_lin.setZero(4,4);
	_K.setZero(10,6*_filter_size);
	_K_lin.setZero(4,3*_filter_size);
	_Lambda = Lambda;
	_Lambda_filt.setZero(6,6);
	_Lambda_filt_lin.setZero(3,3);


}

void RecursiveLeastSquare::init()
{
	_accel_local.setZero();
	_aaccel_local.setZero();
	_avel_local.setZero();
	_g_local.setZero();
	_A_curr.setZero(6,10);
	_A.setZero(6,10); 
	_A_lin_curr.setZero(3,4);
	_A_lin.setZero(3,4);
	_phi.setZero(10);
	_phi_lin.setZero(4); 
	_ft.setZero(6); 
	_FT.setZero(6); 
	_f.setZero();
	_F.setZero(3); 
	_ft_contact.setZero(6); 
	_f_contact.setZero(); 
	_n_measurements = 0;
	_Sigma.setZero(10,10);
	_Sigma_lin.setZero(4,4);
	_K.setZero(10,6*_filter_size);
	_K_lin.setZero(4,3*_filter_size);
	_Lambda_filt.setZero(6,6);
	_Lambda_filt_lin.setZero(3,3);
}


void RecursiveLeastSquare::addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local)
{
	_accel_local  = accel_local;
	_avel_local   = avel_local;
	_aaccel_local = aaccel_local;
	_g_local 	  = g_local;

	if(_linear_case == false)
	{
		getDataMatrix(_A_curr); 
		_ft = force_measurment;
	}
	else
	{
		getDataMatrixLin(_A_lin_curr);
		_f = force_measurment;
	}

	updateParameters();
	updateData();
	
}


void RecursiveLeastSquare::updateData()
{	
	_n_measurements++;
	if(_linear_case == false)
	{
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
			// Eigen::MatrixXd A_temp = _A;
			// Eigen::VectorXd FT_temp = _FT;

			// _A.resize(_n_measurements*6, 10);
			// _FT.resize(_n_measurements*6);

			// _A.topRows((_n_measurements-1)*6) = A_temp;
			// _FT.topRows((_n_measurements-1)*6) = FT_temp;

			// _A.bottomRows(6) = _A_curr;
			// _FT.bottomRows(6) = _ft;
		}
		else if (_n_measurements > _filter_size)
		{
			Eigen::MatrixXd A_temp = _A.topRows((_filter_size-1)*6);
			Eigen::VectorXd FT_temp = _FT.topRows((_filter_size-1)*6);


			_A.bottomRows((_filter_size-1)*6) = A_temp;
			_FT.bottomRows((_filter_size-1)*6) = FT_temp;

			_A.topRows(6) = _A_curr;
			_FT.topRows(6) = _ft; 

			// Eigen::MatrixXd A_temp = _A.bottomRows((_filter_size-1)*6);
			// Eigen::VectorXd FT_temp = _FT.bottomRows((_filter_size-1)*6);

			// _A.bottomRows(6) = _A_curr;
			// _FT.bottomRows(6) = _ft; 

			// _A.topRows((_filter_size-1)*6) = A_temp;
			// _FT.topRows((_filter_size-1)*6) = FT_temp;
		}
	}
	else
	{
		if(_n_measurements == 1)
		{
			_A_lin = _A_lin_curr;
			_F = _f;
		}

		else if(_n_measurements <= _filter_size)
		{
			Eigen::MatrixXd A_temp = _A_lin;
			Eigen::VectorXd F_temp = _F;

			_A_lin.resize(_n_measurements*3, 4);
			_F.resize(_n_measurements*3);

			_A_lin.topRows((_n_measurements-1)*3) = A_temp;
			_F.topRows((_n_measurements-1)*3) = F_temp;

			_A_lin.bottomRows(3) = _A_lin_curr;
			_F.bottomRows(3) = _f;
		}
		else if (_n_measurements > _filter_size)
		{
			std::cout << "error in adding measurement" << _n_measurements << std::endl;
		}
	}
}





void RecursiveLeastSquare::updateParameters()
{
	if (_linear_case==false)
	{
		if (_n_measurements == 0)
		{
			_Sigma = Eigen::MatrixXd::Identity(10,10);

			_Lambda_filt.resize(6*_filter_size, 6*_filter_size);

			for (int i=0; i<_filter_size; i++)
			{
				_Lambda_filt.block(i*6,i*6,6,6) = _Lambda;
			}

		}
		else if (_n_measurements >=_filter_size)
		{	

			// std::cout << "Data Matrix: " << _A << std::endl;
			// std::cout << "FT: " << _FT.transpose() << std::endl;
			// std::cout << "Sigma: " << _Sigma << std::endl;
			// std::cout << "Lambda: " << _Lambda << std::endl;
			_K = _Sigma*_A.transpose()*(_A*_Sigma*_A.transpose()+ _Lambda_filt).inverse();

			// std::cout << "Gain: " << _K << std::endl;
			_Sigma = (Eigen::MatrixXd::Identity(10,10) - _K*_A)*_Sigma;
			// std::cout << "Sigma: " << _Sigma << std::endl;
			_phi = _phi + _K*(_FT - _A*_phi);
			// std::cout << "phi: " << _phi.transpose() << std::endl;
		}
	}
	else
	{
		if (_n_measurements == 0)
		{
			_Sigma_lin = Eigen::MatrixXd::Identity(4,4);

			_Lambda_filt_lin.resize(3*_filter_size, 3*_filter_size);

			for (int i=0; i<_filter_size; i++)
			{
				_Lambda_filt_lin.block(i*3,i*3,3,3) = _Lambda;
			}

		}
		else if (_n_measurements ==_filter_size)
		{
			_K_lin = computeKLin();
			_Sigma_lin = computeSigmaLin();
			_phi_lin = _phi_lin + _K_lin*(_F - _A_lin*_phi_lin);

			_n_measurements = 0;
			_A_lin.setZero(3*_filter_size,4);
			_F.setZero(3*_filter_size);
			_A_lin.resize(3,4);
			_F.resize(3);
		}
	}

}




Eigen::MatrixXd RecursiveLeastSquare::computeK()
{
	Eigen::MatrixXd K = _Sigma*_A.transpose()*(_A*_Sigma*_A.transpose()+ _Lambda_filt).inverse();
	return K;
}
Eigen::MatrixXd RecursiveLeastSquare::computeKLin()
{
	Eigen::MatrixXd K_lin = _Sigma_lin*_A_lin.transpose()*(_A_lin*_Sigma_lin*_A_lin.transpose() + _Lambda_filt_lin).inverse();
	return K_lin;
}

Eigen::MatrixXd RecursiveLeastSquare::computeSigma()
{
	Eigen::MatrixXd Sigma = (Eigen::MatrixXd::Identity(10,10) - _K*_A)*_Sigma;
	return Sigma;
}
Eigen::MatrixXd RecursiveLeastSquare::computeSigmaLin()
{
	Eigen::MatrixXd Sigma_lin = (Eigen::MatrixXd::Identity(4,4) - _K_lin*_A_lin)*_Sigma_lin;
	return Sigma_lin;

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

void RecursiveLeastSquare::getDataMatrixLin(Eigen::MatrixXd& A_data_lin) 
{	
	Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(6,10);
	getDataMatrix(A_full);
	A_data_lin = A_full.block(0,0,3,4);
}




	

Eigen::Vector3d RecursiveLeastSquare::computeContactForce(const Eigen::Vector3d& force_measured,const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local)
{	
	_accel_local  = accel_local;
	_avel_local   = avel_local;
	_aaccel_local = aaccel_local;
	_g_local 	  = g_local;
	_phi_lin	  = phi;

	getDataMatrixLin(_A_lin_curr);

	_f_contact = force_measured - _A_lin_curr * _phi_lin;

	return _f_contact;
}

Eigen::VectorXd RecursiveLeastSquare::computeContactForceTorque(const Eigen::VectorXd& force_torque_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local)
{
	_accel_local  = accel_local;
	_avel_local   = avel_local;
	_aaccel_local = aaccel_local;
	_g_local 	  = g_local;
	_phi     	  = phi;

	getDataMatrixLin(_A_curr);

	_ft_contact = force_torque_measured - _A_curr * _phi;

	return _ft_contact;
}

Eigen::VectorXd RecursiveLeastSquare::getInertialParameterVector()
{
	if(_linear_case==false)
	{
		return _phi;
	}
	else
	{
		return _phi_lin;
	}
}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentDataMatrixStacked()
{
	if(_linear_case==false)
	{
		return _A;
	}
	else
	{
		return _A_lin;
	}
}
Eigen::VectorXd RecursiveLeastSquare::getCurrentInputVectorStacked()
{
	if(_linear_case==false)
	{
		return _FT;
	}
	else
	{
		return _F;
	}
}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentDataMatrix()
{
	if(_linear_case==false)
	{
		return _A_curr;
	}
	else
	{
		return _A_lin_curr;
	}
}
Eigen::VectorXd RecursiveLeastSquare::getCurrentInputVector()
{
	if(_linear_case==false)
	{
		return _ft;
	}
	else
	{
		return _f;
	}
}


Eigen::MatrixXd RecursiveLeastSquare::getCurrentGainMatrix()
{
	if(_linear_case==false)
	{
		return _K;
	}
	else
	{
		return _K_lin;
	}
}


Eigen::MatrixXd RecursiveLeastSquare::getCurrentParameterCovarianceMatrix()
{
	if(_linear_case==false)
	{
		return _Sigma;
	}
	else
	{
		return _Sigma_lin;
	}
}

Eigen::MatrixXd RecursiveLeastSquare::getCurrentNoiseCovarianceMatrix()
{

		return _Lambda;


}

}/* namespace ParameterEstimation */








