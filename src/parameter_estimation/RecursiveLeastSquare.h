
#ifndef PARAMETER_ESTIMATION_RECURSIVE_LEAST_SQUARE_H_
#define PARAMETER_ESTIMATION_RECURSIVE_LEAST_SQUARE_H_

#include <Eigen/Dense>

namespace ParameterEstimation 
{
class RecursiveLeastSquare	
{
public:
	/**
	 * @brief Recursive Least Square estimation
	 * @param filer_size      determines how many matrices should be stacked for one step
	 * @param Lambda 		  measurement noise covariance matrix, size:6x6
	 */
	RecursiveLeastSquare(int filter_size, const Eigen::MatrixXd& Lambda);



	//------------------------------------------------
	// Methods
	//------------------------------------------------


	/**
	* @brief initializes the estimation task
	*/
	void init();

	/**
	* @brief adds measurements to the algorithm, stacks the data matrix and force vector, updates the current estimation of the inertial parameters
	* @param force_measurement		force/torque measurements with respect to force sensor frame
	* @param accel_local 			linear acceleration with respect to force sensor frame
	* @param avel_locel 			angular velocity with respect to force sensor frame
	* @param aaccel_local 			angular acceleration with respect to force sensor frame
	* @param g_local 				gravity with respect to force sensor frame
	*/
	void addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);



	/* 
	* @brief returns  inertial parameter vector (10x1) based in recursive least squares
 	* output(0)	m
 	* output(1)	m*c_x
 	* output(2)	m*c_y
 	* output(3)	m*c_z
 	* output(4)	I_xx,
 	* output(5)	I_xy,
 	* output(6)	I_xz,
 	* output(7)	I_yy,
 	* output(8)	I_yz,
 	* output(9)	I_zz,
 	*/
	Eigen::VectorXd getInertialParameterVector();

	/*
	* @brief computes the contact force based on the previously estimated inertial parameters and current kinematic variables: 
	*        force_contact = force_measured - A_data*phi
	* @param phi 			previously estimated linear inertial parameter
	* @param force_measured measured force vector in force sensor frame
	* @param accel_local 	linear acceleration with respect to force sensor frame
	* @param avel_locel 	angular velocity with respect to force sensor frame
	* @param aaccel_local 	angular acceleration with respect to frame
	* @param g_local 		gravity with respect to force sensor frame
	*
	*/
	Eigen::VectorXd computeContactForceTorque(const Eigen::VectorXd& force_torque_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);
	
	/*
	* @brief returns the current data matrix
	* needs to be called after addData()
	*/
	Eigen::MatrixXd getCurrentDataMatrix();
	/*
	* @brief returns the current force torque vector 
	* needs to be called after addData()
	*/	
	Eigen::VectorXd getCurrentInputVector();

	/*
	* @brief returns the current stacked data matrices
	*/	
	Eigen::MatrixXd getCurrentDataMatrixStacked();

	/*
	* @brief returns the current stacked force/torque vectors
	*/
	Eigen::VectorXd getCurrentInputVectorStacked();


	/*
	* @brief returns the current gain matrix, size 10x6*filter_size
	*/
	Eigen::MatrixXd getCurrentGainMatrix();

	/*
	* @brief returns the current paraneter covariance matrix, size 10x10
	*/
	Eigen::MatrixXd getCurrentParameterCovarianceMatrix();

	/*
	* @brief returns the noise covariance matrix, size 6x6
	*/
	Eigen::MatrixXd getCurrentNoiseCovarianceMatrix();

private:


	/**
	* @brief stacks the data matrix and the force torque vector
	* 		 is called by addData() function
	*/
	void updateData();

	/**
	* @brief updates the current parameter covariance matrix, gain matrix and inertial parameter vector
	* 		 is called by addData() function
	*/
	void updateParameters();

	/** 
	* @brief 	computes data matrix based on 
 	*"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 	*			is called by addData() function
 	* @param 	A_data Matrix to which the data matrix will be written to
 	**/
	void getDataMatrix(Eigen::MatrixXd& A_data);


	Eigen::Vector3d _accel_local; 	//object linear acceleration in sensor frame
	Eigen::Vector3d _aaccel_local; 	//object angular acceleration in sensor frame
	Eigen::Vector3d _avel_local;    //object angular velocity in sensor frame
	Eigen::Vector3d _g_local; 		//gravity vector in sensor frame
	Eigen::MatrixXd _A_curr;		//current data matrix
	Eigen::MatrixXd _A; 			//stack of data matrices
	Eigen::VectorXd _ft; 			//current force/torque measurement
	Eigen::VectorXd _FT; 			//stack of force/torque measurements
	Eigen::VectorXd _phi; 			//estimated inertial parameter vector
	Eigen::VectorXd _ft_contact; 	//computed contact force
	Eigen::MatrixXd _Sigma;			//parameter covariance matrix
	Eigen::MatrixXd _K;				//gain matrix
	Eigen::MatrixXd _Lambda;	   //measurement noise covariance matrix
	Eigen::MatrixXd _Lambda_filt;	//measurement noise covariance matrix, proper dimensions for filter size

	int _n_measurements;			//measurement index
	int _filter_size;		//determines how many measurements will be stacked for one estimation step


};

} /* namespace ParameterEstimation */


#endif //PARAMETER_ESTIMATION_RECURSIVE_LEAST_SQUARE_H_ 
