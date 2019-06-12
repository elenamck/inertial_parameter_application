
#ifndef PARAMETER_ESTIMATION_LEAST_SQUARE_H
#define PARAMETER_ESTIMATION_LEAST_SQUARE_H

#include <Eigen/Dense>

namespace ParameterEstimation

{
class LeastSquare	
{
public:
	/**
	 * @brief Constructor for LeastSquare solution 
	 * following the approach described in:
	 * "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
	 */
	LeastSquare();

	//------------------------------------------------
	// Methods
	//------------------------------------------------
	/**
	* @brief adds measurements to the algorithm, stacks the data matrix and force vector
	* @param force_measurement		force/torque measurements with respect to force sensor frame
	* @param accel_local 			linear acceleration with respect to force sensor frame
	* @param avel_locel 			angular velocity with respect to force sensor frame
	* @param aaccel_local 			angular acceleration with respect to force sensor frame
	* @param g_local 				gravity with respect to force sensor frame
	*/
	void addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);




	/* 
	* @brief returns inertial parameter vector (10x1) based on least squares, uses the Eigen implementation of the SVD 
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
	* @brief computes the contact force/torque based on the previously estimated inertial parameters and current kinematic variables: 
	*        force_contact = force_measured - A_data*phi
	* @param phi 			previously estimated inertial parameter
	* @param force_measured measured force/torque vector in force sensor frame
	* @param accel_local 	linear acceleration with respect to force sensor frame
	* @param avel_locel 	angular velocity with respect to force sensor frame
	* @param aaccel_local 	angular acceleration with respect to frame
	* @param g_local 		gravity with respect to force sensor frame
	*
 	*/
	Eigen::VectorXd computeContactForceTorque(const Eigen::VectorXd& force_torque_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);
	
	/*
	* @brief returns the current stacked data matrices
	*/
	Eigen::MatrixXd getCurrentDataMatrixStacked();

	/*
	* @brief returns the current stacked force/torque vectors
	*/
	Eigen::VectorXd getCurrentInputVectorStacked();

	//------------------------------------------------
	// Methods to find a suitable trajectory for the estimation task
	// see see 01-panda_force_control/03-trajectories/tune_sinusoidal_trajectories.cpp as an example
	//------------------------------------------------

	/*
	* @brief used for determining wheter the trajectory is qualified for the estimation task in terms of the condition number of the trajectory
	* @param accel_local 	linear acceleration with respect to force sensor frame
	* @param avel_locel 	angular velocity with respect to force sensor frame
	* @param aaccel_local 	angular acceleration with respect to frame
	* @param g_local 		gravity with respect to force sensor frame
 	*/
	void addDataConditioning(const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);
	
	/*
	* @brief initialiazes the conditioning task when, to call if suitable trajectory found or potential trajectory failed to meet the requierements (e.g. joint limits)
 	*/
	void initConditioning();

	/*
	* @brief returns the data matrix used for the conditioning task
	* 		 it differs from the data matrix for the estimation task in terms of its dimensions
	*/

	Eigen::MatrixXd getDataMatrixConditioning();

	/*
	* @brief returns the correlation matrix: A_conditioning.transpose()*A_conditioning: 
	* 		 qualification of trajectory for estimation based on its conditioning number
	*/
	Eigen::MatrixXd getCorrelationMatrixConditioning();


private:

	/**
	* @brief stacks the data matrix and the force torque vector
	* 		 is called by addData() function
	*/
	void updateData();

	/** 
	* @brief 	computes data matrix based on 
 	*"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 	*			is called by addData() function
 	* @param 	A_data Matrix to which the data matrix will be written to
 	**/
	void getDataMatrix(Eigen::MatrixXd& A_data);

	void updateDataConditioning();


	Eigen::Vector3d _accel_local; 	//object linear acceleration in force sensor frame
	Eigen::Vector3d _aaccel_local; 	//object angular acceleration in force sensor frame
	Eigen::Vector3d _avel_local;    //object angular velocity in force sensor frame
	Eigen::Vector3d _g_local; 		//gravity vector in force sensor frame
	Eigen::MatrixXd _A_curr;		//current data matrix
	Eigen::MatrixXd _A; 			//stack of data matrices
	Eigen::VectorXd _ft; 			//current force/torque measurement
	Eigen::VectorXd _FT; 			//stack of force/torque measurements
	Eigen::VectorXd _phi; 			//estimated inertial parameter vector
	Eigen::VectorXd _ft_contact; 	//computed contact force torque
	Eigen::MatrixXd _A_conditioning; //data matrix used for finding qualified excictation trajectories
	int _n_measurements;
};

} /* namespace ParameterEstimation */


#endif //PARAMETER_ESTIMATION_LEAST_SQUARE_H 
