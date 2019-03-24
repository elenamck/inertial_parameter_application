#ifndef RECURSIVE_LEAST_SQUARE_H_
#define RECURSIVE_LEAST_SQUARE_H_

#include <eigen3/Eigen/Dense>

class RecursiveLeastSquare	
{
public:
	/**
	 * @brief Recursive Least Square estimation
	 * @param filer_size      determines how many matrices should be stacked for computation
	 * @param Lambda 		  measurement noise covariance matrix, in linear case of size 3x3, else 6x6
	 */
	RecursiveLeastSquare(bool lin, int filter_size, const Eigen::MatrixXd& Lambda);

	//------------------------------------------------
	// Methods
	//------------------------------------------------
	/**
	* @brief adds measurements to the algorithm, stacks the data matrix and force vector, updates the RLS
	* @param force_measurement		force (linear case) or force/torque measurements with respect to force sensor frame
	* @param accel_local 			linear acceleration with respect to force sensor frame
	* @param avel_locel 			angular velocity with respect to force sensor frame
	* @param aaccel_local 			angular acceleration with respect to frame
	* @param g_local 				gravity with respect to force sensor frame
	*/
	void addData(const Eigen::VectorXd& force_measurment, const Eigen::Vector3d& accel_local, const Eigen::Vector3d& avel_local, const Eigen::Vector3d& aaccel_local, const Eigen::Vector3d& g_local);

	/**
	* @brief stacks the matrices based on the given filter size
	*/
	void updateData();

	/**
	* @brief updates the current inertial parameter vector
	*/
	void updateParameters();


	/**
	* @brief 		computes the parameter covariance matrix
	*/
	Eigen::MatrixXd computeSigma();
	/**
	* @brief 		computes the parameter covariance matrix, linear case
	*/
	Eigen::MatrixXd computeSigmaLin();


	/**
	* @brief 		computes the gain matrix
	*/
	Eigen::MatrixXd computeK();
	/**
	* @brief 		computes the gain matrix, linear case
	*/
	Eigen::MatrixXd computeKLin();

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
	Eigen::MatrixXd _Sigma;			//parameter covariance matrix
	Eigen::MatrixXd _Sigma_lin;
	Eigen::MatrixXd _K;				//gain matrix
	Eigen::MatrixXd _K_lin;
	Eigen::MatrixXd _Lambda;		//measurement noise covariance matrix, given
	Eigen::MatrixXd _Lambda_filt;	//measurement noise covariance matrix, proper dimensions for filter size
	Eigen::MatrixXd _Lambda_filt_lin;

	int _n_measurements;			//measurement index
	bool _linear_case;				//flag for linear case
	int _filter_size;


};



#endif //RECURSIVE_LEAST_SQUARE_H_ 