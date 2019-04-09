
#ifndef PARAMETER_ESTIMATION_LEAST_SQUARE_H
#define PARAMETER_ESTIMATION_LEAST_SQUARE_H

#include <Eigen/Dense>

namespace ParameterEstimation

{
class LeastSquare	
{
public:
	/**
	 * @brief Constructor that takes the link name and the position of where to compute the velocities/accelerations.
	 * @param robot           A pointer to a Sai2Model object for the robot 
	 * @param link_name       The name of the link in the robot at which the kinematics are to be computed
	 * @param pos_in_link     The position in link where the kinematics are to be comuted
	 */
//todo: constructor which determines the algorithm?
	LeastSquare(bool lin);

	//------------------------------------------------
	// Methods
	//------------------------------------------------
	/**
	* @brief adds measurements to the algorithm
	* @param force_measurement		force (linear case) or force/torque measurements with respect to force sensor frame
	* @param accel_local 			linear acceleration with respect to force sensor frame
	* @param avel_locel 			angular velocity with respect to force sensor frame
	* @param aaccel_local 			angular acceleration with respect to frame
	* @param g_local 				gravity with respect to force sensor frame
	*/
	void addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);

	/**
	* @brief updates the inertial parameter vector based on the chosen algorithm
	* is called by addData function
	*/
	void updateData();

	/** 
	* @brief 	computes data matrix based on 
 	*"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 	* @param 	A_data Matrix to which the data matrix will be written
 	**/
	void getDataMatrix(Eigen::MatrixXd& A_data);

	/**  
	* @brief 	computes linear data matrix based on 
 	* "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 	* @param 	A_data_lin Matrix to which the linear data matrix will be written
 	**/
	void getDataMatrixLin(Eigen::MatrixXd& A_data_lin);

	/* 
	* @brief returns inertial parameter vector (4x1) in linear case and full inertial parameter vector (10x1) else
	*		 based on least squares 
 	* 		 linear case: mass, mass*coordinates of center of mass
 	*		 else: mass, mass*coordinates of center of mass, elements of the inertia matrix
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
	Eigen::Vector3d computeContactForce(const Eigen::Vector3d& force_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);

	Eigen::VectorXd computeContactForceTorque(const Eigen::VectorXd& force_torque_measured, const Eigen::VectorXd& phi, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);

	void addDataConditioning(const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);
	void updateDataConditioning();
	void initConditioning();
	Eigen::MatrixXd getDataMatrixConditioning();
	Eigen::MatrixXd getCorrelationMatrixConditioning();
	Eigen::MatrixXd getCurrentDataMatrixStacked();
	Eigen::VectorXd getCurrentInputVectorStacked();

private:

	Eigen::Vector3d _accel_local; 	//object linear acceleration in sensor frame
	Eigen::Vector3d _aaccel_local; 	//object angular acceleration in sensor frame
	Eigen::Vector3d _avel_local;    //object angular velocity in sensor frame
	Eigen::Vector3d _g_local; 		//gravity vector in sensor frame
	Eigen::MatrixXd _A_curr;		//current data matrix
	Eigen::MatrixXd _A; 			//stack of data matrices
	Eigen::MatrixXd _A_lin_curr; 	//current linear data matrix
	Eigen::MatrixXd _A_lin; 		//stack of linear data matrices
	Eigen::VectorXd _ft; 			//current force/torque measurement
	Eigen::VectorXd _FT; 			//stack of force/torque measurements
	Eigen::Vector3d _f; 			//current force measurement
	Eigen::VectorXd _F; 			//stack of force measurements
	Eigen::VectorXd _phi; 			//estimated inertial parameter vector
	Eigen::VectorXd _phi_lin; 		//estimated inertial parameter vector
	Eigen::VectorXd _ft_contact; 	//computed contact force
	Eigen::Vector3d _f_contact; 	//computed contact force
	Eigen::MatrixXd _A_conditioning;
	int _n_measurements;
	bool _linear_case;				//flag for linear case
};

} /* namespace ParameterEstimation */


#endif //PARAMETER_ESTIMATION_LEAST_SQUARE_H 
