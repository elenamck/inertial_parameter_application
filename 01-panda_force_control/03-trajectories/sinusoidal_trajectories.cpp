#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "trajectories/JointSpaceSinusodial.h"
#include "filters/MeanFilter.h"




#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>
#include <chrono>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../../resources/01-panda_force_control/panda_arm.urdf";
const string robot_name = "FRANKA-PANDA";

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

const bool inertia_regularization = true;
// redis keys:
// - write:
string JOINT_TORQUES_COMMANDED_KEY;
string LOCAL_GRAVITY_KEY;
string EE_FORCE_SENSOR_FORCE_UNBIASED_KEY;

// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string EE_FORCE_SENSOR_FORCE_KEY;
string ACCELEROMETER_DATA_KEY;
string GYROSCOPE_DATA_KEY;


// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

// - offline processing
string LINEAR_ACCELERATION_LOCAL_KEY;
string ANGULAR_VELOCITY_LOCAL_KEY;
string EE_FORCE_SENSOR_KEY;

// - inertial parameters
string INERTIAL_PARAMS_KEY;


// - kinematics:
string POSITION_KIN_KEY;
string LINEAR_VEL_KIN_KEY;
string LINEAR_ACC_KIN_KEY;
string QUATERNION_KIN_KEY;
string ANGULAR_VEL_KIN_KEY;
string ANGULAR_ACC_KIN_KEY;

// - kalman filters
string POSITION_KF_KEY;
string LINEAR_VEL_KF_KEY;
string LINEAR_ACC_KF_KEY;
string QUATERNION_EKF_KEY;
string ANGULAR_VEL_EKF_KEY;
string ANGULAR_ACC_EKF_KEY;

string ANGULAR_VEL_TEST;
string LINEAR_ACC_TEST;

string DATA_MATRIX_CURR;
string FT_CURR;

string FORCE_VIRTUAL_DES_KEY;

string JOINT_ANGLE_INPUTS_KEY;
string JOINT_VELOCITIES_INPUTS_KEY;

string POS_EE_KEY;


#define  GOTO_INITIAL_CONFIG 	 0
#define  SINUSODIAL				 1
#define	 REST 					 2




int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force_des";

		POS_EE_KEY = "sai2::DemoApplication::Panda::kinematics::pos::sensor";



		// "sai2::DemoApplication::Panda::simulation::virtual_force";
		EE_FORCE_SENSOR_FORCE_UNBIASED_KEY = "sai2::DemoApplication::Panda::controller::ftunbiased";

		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";		

		//InertialParameters
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";


		//Kinematics
		POSITION_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
		LINEAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
		LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::accel";

		QUATERNION_KIN_KEY =  "sai2::DemoApplication::Panda::kinematics::ori::quats";
		ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::avel";
		ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::aaccel";

		DATA_MATRIX_CURR = "sai2::DemoApplication::Panda::estimation::current_data_matrix";
		FT_CURR = "sai2::DemoApplication::Panda::estimation::current_data_ft";


		JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";


	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
		EE_FORCE_SENSOR_FORCE_UNBIASED_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::controller::ftunbiased";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";

		ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";      
		GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";    

		POSITION_KIN_KEY    = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::pos";
		LINEAR_VEL_KIN_KEY  = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
		LINEAR_ACC_KIN_KEY  = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel";

		QUATERNION_KIN_KEY  = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::ori::quats";
		ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::avel";
		ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::aaccel";

		POSITION_KF_KEY     = "sai2::DemoApplication::FrankaPanda::Clyde::KF::pos";
		LINEAR_VEL_KF_KEY   = "sai2::DemoApplication::FrankaPanda::Clyde::KF::vel";
		LINEAR_ACC_KF_KEY   = "sai2::DemoApplication::FrankaPanda::Clyde::KF::accel";

		QUATERNION_EKF_KEY  = "sai2::DemoApplication::FrankaPanda::Clyde::EKF::ori::quats";
		ANGULAR_VEL_EKF_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::EKF::avel";
		ANGULAR_ACC_EKF_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::EKF::aaccel";

		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::controller::phi";

		ANGULAR_VEL_TEST = "sai2::DemoApplication::FrankaPanda::Clyde::controller::avel_test";
		LINEAR_ACC_TEST = "sai2::DemoApplication::FrankaPanda::Clyde::controller::accel_test";
		
		
	}

	
    // start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);



	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// read from Redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	robot->updateModel();


	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);



	//Controllers
	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	// pos ori controller
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0,0,0.106);


	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/4;
	joint_task->_kp = 50;
	joint_task->_kv = 11;
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	// desired_initial_configuration << 0, 10, 0, -125, 0, 135, 0;


	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;


	Matrix3d current_orientation = Matrix3d::Zero();



	Matrix3d R_link = Matrix3d::Zero();

	//For Inertial Parameter Estimation
	bool linear_case = true;
	bool non_linear_case = false;
	Matrix3d Lambda_lin = 0.01*Matrix3d::Identity();
	MatrixXd Lambda = 0.014 * MatrixXd::Identity(6,6);

	// auto RLS = new ParameterEstimation::RecursiveLeastSquare(linear_case,4,Lambda_lin);
	auto RLS = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,5,Lambda);
	auto LS = new ParameterEstimation::LeastSquare(false);


	
	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	VectorXd phi_LS = VectorXd::Zero(10); //inertial parameter vector

	Vector3d force_sensed = Vector3d::Zero();
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();
	Matrix3d inertia_tensor_LS = Matrix3d::Zero();
	Vector3d center_of_mass_LS = Vector3d::Zero();
	MatrixXd A_curr = MatrixXd::Zero(6,10);
	VectorXd ft_curr = VectorXd::Zero(6);



	//position and orientation
	Vector3d pos_kin =  Vector3d::Zero();
	Vector3d vel_kin =  Vector3d::Zero();
	Vector3d accel_kin = Vector3d::Zero();


	Matrix3d orientation = Matrix3d::Zero();
	Quaterniond ori_quat_kin_aux = Quaterniond(0,0,0,0);
	VectorXd ori_quat_kin = VectorXd::Zero(4);
	Vector3d avel_kin = Vector3d::Zero();
	Vector3d aaccel_kin = Vector3d::Zero();

	//Kalman Filter
	int n_kf = 9;
	int m_kf = 6;
	double dt = 1.0/500;
	MatrixXd A = MatrixXd::Zero(n_kf,n_kf); // System dynamics matrix
    MatrixXd C = MatrixXd::Zero(m_kf,n_kf); // Output matrix
    MatrixXd Q = MatrixXd::Zero(n_kf,n_kf); // Process noise covariance
  	MatrixXd R = MatrixXd::Zero(m_kf,m_kf); // Measurement noise covariance
    MatrixXd P = MatrixXd::Identity(n_kf,n_kf); // Estimate error covariance

    A.block(0,0,3,3) = Matrix3d::Identity();
    A.block(0,3,3,3) = dt * Matrix3d::Identity();
    A.block(0,6,3,3) = 0.5*dt*dt*Matrix3d::Identity();
    A.block(3,0,3,3) = Matrix3d::Zero();
    A.block(3,3,3,3) = Matrix3d::Identity();
    A.block(3,6,3,3) = dt*Matrix3d::Identity();
    A.block(6,0,3,3) = Matrix3d::Zero();
    A.block(6,3,3,3) = Matrix3d::Zero();
    A.block(6,6,3,3) = Matrix3d::Identity();

    C.block(0,0,3,3) = Matrix3d::Identity();
    C.block(0,3,3,6) = MatrixXd::Zero(3,6);
    C.block(3,0,3,6) = MatrixXd::Zero(3,6);
    C.block(3,6,3,3) = Matrix3d::Identity();

    // Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4; //-------------------TODO check which are better!--------------------------
    // R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-3, 1.0e-3, 1.0e-3;

    // Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6;
    // R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-2, 1.0e-2, 1.0e-2;

    Q.diagonal() << 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-7, 1.0e-7, 1.0e-7, 1e-7, 1.0e-7, 1.0e-7;
    R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1e-2, 1e-2, 1e-2;

	auto kalman_filter = new KalmanFilters::KalmanFilter(dt, A, C, Q, R, P);

    VectorXd x0 = VectorXd::Zero(n_kf);
    double t0 = 0;
    kalman_filter->init(t0, x0);
    VectorXd y = VectorXd::Zero(m_kf);
    VectorXd kf_states = VectorXd::Zero(n_kf);
    Vector3d accel_aux = Vector3d::Zero();


    //Extended Kalman Filter
    int n_ekf = 10;
    int m_ekf = 7;

    MatrixXd C_ekf = MatrixXd::Zero(m_ekf,n_ekf); //Output matrix
  	MatrixXd Q_ekf = MatrixXd::Identity(n_ekf,n_ekf); //Process noise covariance
  	MatrixXd R_ekf = MatrixXd::Identity(m_ekf,m_ekf); // Measurement noise covariance
  	MatrixXd P_ekf = MatrixXd::Identity(n_ekf,n_ekf); // Estimate error covariance

  	C_ekf = MatrixXd::Identity(m_ekf, n_ekf);

  	//Q_ekf.diagonal() << 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-7, 1.0e-7, 1.0e-7;
  	Q_ekf.diagonal() << 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6;
  	R_ekf.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1e-5, 1e-5, 1e-5;
  	VectorXd y_ekf = VectorXd::Zero(m_ekf);

	auto extended_kalman_filter = new KalmanFilters::QuaternionBasedEKF( dt, C_ekf, Q_ekf, R_ekf, P_ekf);
	const int window_size = 2;
	auto mean_accel = new Filters::MeanFilter(window_size,3);
	auto mean_avel = new Filters::MeanFilter(window_size,3);
	auto mean_aaccel = new Filters::MeanFilter(window_size,3);
	auto mean_g_local = new Filters::MeanFilter(window_size,3);
	auto mean_force_moment = new Filters::MeanFilter(window_size,6);

	VectorXd accel_local_mean = VectorXd::Zero(3);
	VectorXd avel_local_mean = VectorXd::Zero(3);
	VectorXd aaccel_local_mean = VectorXd::Zero(3);
	VectorXd g_local_mean = VectorXd::Zero(3);
	VectorXd force_moment_mean = VectorXd::Zero(6);



  	

  	VectorXd x0_ekf = VectorXd::Zero(n_ekf);
  	extended_kalman_filter->init(t0, x0_ekf);
  	VectorXd ekf_states = VectorXd::Zero(n_ekf);
  	Vector3d avel_aux = Vector3d::Zero();

  	//Kalman Filter States
  	Vector3d pos_kf = Vector3d::Zero();
  	Vector3d vel_kf = Vector3d::Zero();
  	VectorXd ori_quat_kf = VectorXd::Zero(4);

    //Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;

	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	ifstream bias;
	bias.open("../02-utilities/FT_data1.txt");
	if (!bias)  
	{                     // if it does not work
        cout << "Can't open Data!" << endl;
    }
    else
    {
    	
    	for (int row ; row<6; row++)
    	{
    		double value = 0.0;
    		bias >> value;
    		force_torque_bias(row) = value;
    	}
    cout << "bias read" << force_torque_bias << endl;
    bias.close();
	}
	
	int state = GOTO_INITIAL_CONFIG;

	//for sinusodial trajectories


	int axis = 4; 
	int N = 3; 
	double w_s = 1000;
	double w_f = 0.4; 
	// double w_f = 0.8; 
	VectorXd q_des = VectorXd::Zero(axis);
	VectorXd dq_des = VectorXd::Zero(axis);
	VectorXd a = VectorXd::Zero(N*axis);
	VectorXd b = VectorXd::Zero(N*axis);
	// a << 0.358813, -0.437701,  0.982793,  0.604349,  0.225106,  0.850517, -0.408695,  0.570985,   1.00535, -0.591772, -0.388226, -0.213033;
	// b << -0.588637, -0.00925619,    0.532357,     1.02669,    0.953303,   -0.672674,   -0.392542 ,   0.490234,    -1.04557 ,   0.996562 ,  -0.343163   , 0.438637;
	// a << 0, -0.437701,  1.582793,  0,  0.225106,  0.850517, -0.408695, 0;
	// b << -0.588637, 0,    0.0532357,    0,    0.953303,   -1.672674,   0, 0.490234 ;
 // a << 0.974478 , -0.769762 ,  0.370534 , -0.327473 ,  0.783655 , -0.307155 ,  0.200041,   0.891952  , 0.685765,  -0.689094   , 0.67666 ,-0.0599719,  -0.192114,   0.836484 ,-0.0368847;
// b <<-0.0587619 , -0.637884  , 0.447126 ,  0.343481 , -0.814954  , 0.554182  , 0.574948  , 0.315083 , 0.0014389,  -0.165644 ,-0.0166339  ,-0.758012  , 0.942208 ,  -0.90314 , -0.374112;


// a << -0.934053, 	0.554726, 	0.147692;	
// b << 0.157019, 	0.822921, 	-0.102832;
// a << -0.307137, 	0.699636, 	-0.0966319, 	-0.0140335, 	-0.637647, 	0.435327, 	-0.221199, 	-0.42967, 	0.254573, 	-0.404099, 	-0.665026, 	-0.379811, 	0.435619, 	-0.391179, 	-0.11536;
// b << -0.238391, 	0.418179, 	0.248428, 	0.322399, 	0.2656, 	-0.229715, 	-0.216378, 	0.338296, 	-0.588994, 	0.66866, 	0.690936, 	0.642729, 	-0.272505, 	-0.518133, 	-0.100065;
	
	a <<  -0.0685063, 	-0.266057, 	0.363782, 	0.49263, 	-0.229633, 	0.581041, 	0.243644, 	0.158148, 	-0.289556, 	-0.0398136, 	-0.622877, 	-0.327384;	



// b <<0.164893, 	0.286424, 	-0.270565, 	0.686873, 	-0.102795, 	0.343893, 	0.0232298, 	-0.136117, 	-0.589154, 	0.182915, 	0.389056, 	0.473262;

	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	VectorXd desired_initial_configuration_trunc = VectorXd::Zero(axis);
	desired_initial_configuration_trunc = desired_initial_configuration.tail(axis);
	joint_trajectory->init(desired_initial_configuration_trunc);
	unsigned int long trajectory_counter = 0;


	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
	// create timer
	chrono::high_resolution_clock::time_point t_start;
	chrono::duration<double> t_elapsed;
	Vector3d pos_in_ee = Vector3d::Zero();
	Affine3d T_link;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();


		// read from Redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);

		//Kinematics 
		robot->position(pos_kin, link_name, Vector3d::Zero());
		robot->linearVelocity(vel_kin, link_name);
		robot->linearAcceleration(accel_kin,link_name, Vector3d::Zero());

		robot->rotation(R_link,link_name);
		robot->angularVelocity(avel_kin,link_name);
		robot->angularAcceleration(aaccel_kin,link_name);	
		robot->transform(T_link, link_name);
		g_local = T_link.inverse()*robot->_world_gravity;
		// g_local = T_link.inverse()*robot->_world_gravity;

		// pos_in_ee = T_link.inverse()*pos_kin;	
		vel_kin = T_link.inverse()*vel_kin;
		accel_kin = T_link.inverse()*accel_kin;
		accel_kin += g_local;

		ori_quat_kin_aux = R_link;
		ori_quat_kin_aux.normalize();
		ori_quat_kin << ori_quat_kin_aux.w(), ori_quat_kin_aux.vec();
		avel_kin = T_link.inverse()*avel_kin;
		aaccel_kin = T_link.inverse()*aaccel_kin;
		
		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);

			accel_local = accel_kin;
			avel_local = avel_kin;
			aaccel_local = aaccel_kin;
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
			 	robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);


			force_moment -= force_torque_bias;
			accel_kin -= g_local;

			accel_aux = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			accel_aux = R_acc_in_ft*accel_aux;
			accel_aux *= 9.81;
			accel_aux += g_local;
			y << pos_kin, accel_aux;
            kalman_filter->update(y);
            kf_states = kalman_filter->state();
            pos_kf << kf_states(0), kf_states(1), kf_states(2);
            vel_kf << kf_states(3), kf_states(4), kf_states(5);
            accel_local << kf_states(6), kf_states(7), kf_states(8);

            avel_aux = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
            avel_aux = R_acc_in_ft*avel_aux;
            avel_aux *= M_PI/180;

            //try this
            ori_quat_kin_aux = R_link;
            ori_quat_kin << ori_quat_kin_aux.w(), ori_quat_kin_aux.vec();
            avel_aux = R_link*avel_aux;

			y_ekf << ori_quat_kin, avel_aux;
			extended_kalman_filter-> update(y_ekf);
			ekf_states = extended_kalman_filter->state();
			ori_quat_kf << ekf_states(0), ekf_states(1), ekf_states(2), ekf_states(3);
			avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
			avel_local = R_link.transpose()*avel_local;
			aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);
			aaccel_local = R_link.transpose()*aaccel_local;
			avel_aux = R_link.transpose()*avel_aux;

		}

		mean_accel->process(accel_local_mean, accel_local);
		mean_avel->process(avel_local_mean, avel_local);
		mean_aaccel->process(aaccel_local_mean, aaccel_local);
		mean_g_local->process(g_local_mean, g_local);
		mean_force_moment->process(force_moment_mean, force_moment);


			RLS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
			phi_RLS = RLS->getInertialParameterVector();
			center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			A_curr = RLS->getCurrentDataMatrix();
			ft_curr = RLS->getCurrentInputVector(); 


		




		if(state == GOTO_INITIAL_CONFIG)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;
			cout << "desired velocity" << joint_task->_desired_velocity.transpose() << endl;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();		
				joint_task->_desired_velocity.tail(axis) = VectorXd::Zero(axis);
			    // joint_task->_goal_position(6) = 0.8*M_PI;

			    state = SINUSODIAL;
				
			}
		}

		else if(state == SINUSODIAL)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_trajectory->update(trajectory_counter);
			q_des = joint_trajectory->getJointAngles();
			dq_des = joint_trajectory->getJointVelocities();
			joint_task->_goal_position.tail(axis) = q_des;
			// joint_task->_desired_velocity.tail(axis)= dq_des;

			if (controller_counter%2 == 0)
			{
				LS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
			}

			if(controller_counter%1000==0)
		{
			cout << "estimated mass: \n" << phi_RLS(0) << "\n";
		    cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << "\n";
		    cout << "estimated Inertia: \n" << inertia_tensor_RLS << "\n";

		}

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			
			trajectory_counter++;

			if ((trajectory_counter/w_s) >= 2*M_PI/w_f)
			{
				
				phi_LS = LS->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				cout << "estimated mass LS: \n" << phi_LS(0) << "\n";
		   		cout << "estimated center of mass LS: \n" << 	center_of_mass_LS.transpose() << "\n";
		    	cout << "estimated Inertia LS: \n" << inertia_tensor_LS << "\n";

		    	// cout << "datamatrix: \n" << LS->getCurrentDataMatrixStacked() << endl;
		    	// cout << "FT Vector: \n" << LS->getCurrentInputVectorStacked() << endl;
		    	



		    	state = REST;
		    	// joint_task->_goal_position = desired_initial_configuration;
			}



		}


		else if(state == REST)
		{

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			LS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques   + coriolis;

			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			// if(config_error.norm() < 0.1)
			// {		
			// 	phi_LS = LS->getInertialParameterVector();
			// 	center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
			// 	inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			// 	cout << "estimated mass LS: \n" << phi_LS(0) << "\n";
		 //   		cout << "estimated center of mass LS: \n" << 	center_of_mass_LS.transpose() << "\n";
		 //    	cout << "estimated Inertia LS: \n" << inertia_tensor_LS << "\n";
		 //    	break;
			// }
	}

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);

		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS);

		redis_client.setEigenMatrixDerived(POSITION_KIN_KEY, pos_kin);
		redis_client.setEigenMatrixDerived(LINEAR_VEL_KIN_KEY, vel_kin);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KIN_KEY, accel_kin);
		redis_client.setEigenMatrixDerived(QUATERNION_KIN_KEY, ori_quat_kin);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KIN_KEY, avel_kin);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KIN_KEY, aaccel_kin);

		redis_client.setEigenMatrixDerived(POSITION_KF_KEY, pos_kf);
		redis_client.setEigenMatrixDerived(LINEAR_VEL_KF_KEY, vel_kf);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KF_KEY, accel_local);
		redis_client.setEigenMatrixDerived(QUATERNION_EKF_KEY, ori_quat_kf);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_EKF_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_EKF_KEY, aaccel_local);


		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_UNBIASED_KEY, force_moment);

		redis_client.setEigenMatrixDerived(ANGULAR_VEL_TEST, avel_aux);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_TEST, accel_aux);
		redis_client.setEigenMatrixDerived(JOINT_ANGLE_INPUTS_KEY, q_des);

		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_INPUTS_KEY, dq_des);
		redis_client.setEigenMatrixDerived(POS_EE_KEY, pos_in_ee);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Loop run time  : " << end_time << " seconds\n";
    cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	cout << "the estimated mass is: \n" << phi_RLS(0) << endl;
    cout << "the estimated center of mass is: \n" << center_of_mass_RLS.transpose() << endl;
    cout << "the estimated inertia tensor is: \n" << inertia_tensor_RLS << endl;

    return 0;

}



