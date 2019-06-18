 #include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "trajectories/JointSpaceSinusodial.h"
#include "filters/SecOrderLowPass.hpp"

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

const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "FRANKA-PANDA";

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

//flags for logging
const bool logging_estimation_inputs = false;
// const bool logging_estimation_inputs = true;
const bool logging_estimation_outputs = false;
// const bool logging_estimation_outputs = true;

const bool logging_model_kinematics = false;
// const bool logging_model_kinematics = true;

// const bool logging_joint_trajectory = false;
const bool logging_joint_trajectory = true;


const bool inertia_regularization = true;
// redis keys:
// - write:
string JOINT_TORQUES_COMMANDED_KEY;

// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_ACCELERATIONS_KEY; //if simulation
string EE_FORCE_SENSOR_FORCE_KEY;
string ACCELEROMETER_DATA_KEY;
string GYROSCOPE_DATA_KEY;
// - read estimation inputs in simulation case:
string LINEAR_ACC_KEY;
string ANGULAR_VEL_KEY;
string ANGULAR_ACC_KEY;


// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

// - logging:
// - inputs
string LOCAL_GRAVITY_KEY;

string LINEAR_ACC_LP_KEY;
string ANGULAR_VEL_LP_KEY;
string ANGULAR_ACC_LP_KEY;
string FORCE_TORQUE_LP_KEY;

string LINEAR_ACC_KF_KEY;
string ANGULAR_VEL_KF_KEY;
string ANGULAR_ACC_KF_KEY;

// - outputs:
string INERTIAL_PARAMS_LP_KEY;
string INERTIAL_PARAMS_KF_KEY;

// - kinematics
string POSITION_KIN_KEY;
string LINEAR_VEL_KIN_KEY;
string LINEAR_ACC_KIN_KEY;
string ORIENTATION_KIN_KEY;
string ANGULAR_VEL_KIN_KEY;
string ANGULAR_ACC_KIN_KEY;

// - sinusoidal trajectory
string JOINT_ANGLE_INPUTS_KEY;
string JOINT_VELOCITIES_INPUTS_KEY;
string JOINT_ACCELERATIONS_INPUTS_KEY;


#define  GOTO_INITIAL_CONFIG 	 0
#define  SINUSOIDAL 			 1
#define	 REST 					 2

int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		JOINT_ACCELERATIONS_KEY = "sai2::DemoApplication::Panda::sensors::ddq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

		LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";

 		LOCAL_GRAVITY_KEY = "sai2::DemoApplication::Panda::simulation::g_local";
		JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";
		JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::ddq";

	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";

		ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";      
		GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";

		JOINT_ANGLE_INPUTS_KEY = "sai2::FrankaPanda::Clyde::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::FrankaPanda::Clyde::desired::dq";
		JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::FrankaPanda::Clyde::desired::ddq";    

		if(logging_estimation_inputs)
		{
			LINEAR_ACC_LP_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::linear_acc::lp";
			ANGULAR_VEL_LP_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::angular_vel::lp";
			ANGULAR_ACC_LP_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::angular_acc::lp";
			FORCE_TORQUE_LP_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::force_torque::lp";

			LINEAR_ACC_KF_KEY= "sai2::FrankaPanda::Clyde::logging::estimation_inputs::linear_acc::kf";
			ANGULAR_VEL_KF_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::angular_vel::kf";
			ANGULAR_ACC_KF_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::angular_acc::kf";

			LOCAL_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_inputs::local_gravity";
		}

		if(logging_estimation_outputs)
		{
			INERTIAL_PARAMS_LP_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_outputs::lp";
			INERTIAL_PARAMS_KF_KEY = "sai2::FrankaPanda::Clyde::logging::estimation_outputs::kf";
		}

		if(logging_model_kinematics)
		{
			POSITION_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::pos";
			LINEAR_VEL_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::vel";
			LINEAR_ACC_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::accel";
			ORIENTATION_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::ori";
			ANGULAR_VEL_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::avel";
			ANGULAR_ACC_KIN_KEY = "sai2::FrankaPanda::Clyde::logging::model_kinematics::aaccel";
		}

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
	if (flag_simulation)
	{
		robot->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCELERATIONS_KEY);
	}


	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	//Controllers
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.2);



	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI / 4;
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;


	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	//For Inertial Parameter Estimation
	//Measurement noise covariance matrix, values based on typical noise 
	MatrixXd Lambda = MatrixXd::Zero(6,6);
	Lambda   << 0.035, 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ,
				0.0  ,0.035, 0.0 , 0.0  , 0.0  , 0.0  ,
				0.0  , 0.0 , 0.15, 0.0  , 0.0  , 0.0  ,
				0.0  , 0.0 , 0.0 , 0.002, 0.0  , 0.0  ,
				0.0  , 0.0 , 0.0 , 0.0  , 0.002, 0.0  ,
				0.0  , 0.0 , 0.0 , 0.0  , 0.0  , 0.001;

	auto RLS_KF = new ParameterEstimation::RecursiveLeastSquare(6, Lambda);
	auto RLS_LP = new ParameterEstimation::RecursiveLeastSquare(6, Lambda);
	auto RLS = new ParameterEstimation::RecursiveLeastSquare(6, Lambda);

	//estimated inertial parameters
	VectorXd phi_RLS_KF = VectorXd::Zero(10); 
	VectorXd phi_RLS_LP = VectorXd::Zero(10);
	VectorXd phi_RLS = VectorXd::Zero(10);


	//inertial parameter input
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame


	Vector3d accel_aux_imu = Vector3d::Zero(); //sensor data, IMU frame
	Vector3d avel_aux_imu = Vector3d::Zero(); //sensor data, IMU frame
	Vector3d accel_aux = Vector3d::Zero(); //sensor data FT frame
	Vector3d avel_aux = Vector3d::Zero(); //sensor data FT frame

	Vector3d accel_lp_filtered = Vector3d::Zero();
	Vector3d avel_lp_filtered = Vector3d::Zero();
	Vector3d aaccel_lp = Vector3d::Zero();
	Vector3d aaccel_lp_filtered = Vector3d::Zero();
	VectorXd force_moment_lp_filtered = VectorXd::Zero(6);




	//Kalman Filter
	int n_kf = 9;
	int m_kf = 6;
	double dt = 1.0/control_freq;
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

    
    Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4;
    R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-2, 1.0e-2, 1.0e-2;
    //if possiple try both
    // Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6;
    // R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-3, 1.0e-3, 1.0e-3;

    auto kalman_filter = new KalmanFilters::KalmanFilter(dt, A, C, Q, R, P);


    VectorXd x0 = VectorXd::Zero(n_kf);
    double t0 = 0;
    kalman_filter->init(t0, x0);
    VectorXd kf_states = VectorXd::Zero(n_kf);

	VectorXd y = VectorXd::Zero(m_kf); //KF input: position and accelerometer data
    Vector3d current_position_aux = Vector3d::Zero(); //endeffektor with resprect to base link
    Vector3d current_position = Vector3d::Zero(); //endeffektor with resprect to last link
    Vector3d accel_kf = Vector3d::Zero(); //kalman filtered acceleration

	

	//Extended Kalman Filter
	Quaterniond q_eff = Quaterniond(0,0,0,0);
    VectorXd q_eff_aux = VectorXd::Zero(4);

    int n_ekf = 10;
    int m_ekf = 7;

    MatrixXd C_ekf = MatrixXd::Zero(m_ekf,n_ekf); //Output matrix
  	MatrixXd Q_ekf = MatrixXd::Identity(n_ekf,n_ekf); //Process noise covariance
  	MatrixXd R_ekf = MatrixXd::Identity(m_ekf,m_ekf); // Measurement noise covariance
  	MatrixXd P_ekf = MatrixXd::Identity(n_ekf,n_ekf); // Estimate error covariance

  	C_ekf = MatrixXd::Identity(m_ekf, n_ekf);

  	Q_ekf.diagonal() << 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-7, 1.0e-7, 1.0e-7;
  	R_ekf.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1e-1, 1e-1, 1e-1;
  	VectorXd y_ekf = VectorXd::Zero(m_ekf);

  	auto extended_kalman_filter = new KalmanFilters::QuaternionBasedEKF( dt, C_ekf, Q_ekf, R_ekf, P_ekf);

  	VectorXd x0_ekf = VectorXd::Zero(n_ekf);
  	extended_kalman_filter->init(t0, x0_ekf);
  	VectorXd ekf_states = VectorXd::Zero(n_ekf);
  	Vector3d avel_kf = Vector3d::Zero();
  	Vector3d aaccel_kf = Vector3d::Zero();

    //Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;
	Matrix3d R_link = Matrix3d::Zero();


	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	ifstream bias;

	bias.open("../../../02-utilities/FT_data1.txt");
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
    cout << "bias read" << force_torque_bias.transpose() << endl;
    bias.close();
	}
	//for sinusodial trajectories
	int axis = 4;
	int N = 3;
	double w_s = control_freq;
	double w_f = 1.2;
	VectorXd q_des = VectorXd::Zero(axis);
	VectorXd dq_des = VectorXd::Zero(axis);
	VectorXd ddq_des = VectorXd::Zero(axis);

	VectorXd a = VectorXd::Zero(N*axis);
	VectorXd b = VectorXd::Zero(N*axis);
	// a << -0.0286409,         0.0322926,       0.47785,         -0.571294,       0.0973072,       -0.10507,        -0.194213,       -0.327815,       0.261298,        -0.659976,       0.634429,        0.0897043;
	// a<< 0.259086 ,-.00621783   , 0.429696 ,  -0.728262,   -0.780967  ,  -0.23069 ,  -0.178586 ,   0.267967 ,  -0.723327  ,  0.641493  , -0.304355  , -0.505646;
a <<  -0.0599598  , -0.237076   , 0.172744 ,   0.103608 ,  -0.156004  ,  0.255555  ,  0.150687  , -0.265947 ,   0.155196 ,0.000679095 ,  -0.214335 ,  -0.258348;
b <<   0.229282 ,    0.2577 ,-0.0939376  , 0.299982 ,-0.0762449 ,  0.208045  ,-0.192033, -0.0500351, -0.0358087  , 0.219918  , 0.160957  ,-0.196047;
//results in kappa: 10.0081
// ft data exists
	// a << 0.259086 ,-0.00621783   , 0.429696 ,  -0.728262,   -0.780967  ,  -0.23069 ,  -0.178586 ,   0.267967 ,  -0.723327  ,  0.641493  , -0.304355  , -0.505646;

// a << -0.0271468, 	0.0998505, 	-0.0669353, 	-0.410767, 	0.354131, 	-0.13737, 	0.330701, 	0.315941, 	0.276246, 	0.578735, 	-0.637234, 	-0.348602; 	

	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	VectorXd desired_initial_configuration_trunc = VectorXd::Zero(axis);
	desired_initial_configuration_trunc = desired_initial_configuration.tail(axis);
	unsigned int long trajectory_counter = 0;
	int trajectory_counter_multiple = 1; //how many times should trajectory be repeated



	
	
	double fc = 10;
	//time constant and damping, for lowpassfilter
	double tau = 1/(2*M_PI*fc);
	double damping = 0.8;


	auto force_moment_low_pass_filter = new am2b::SecOrderLowPass<VectorXd>(VectorXd::Zero(6));
	force_moment_low_pass_filter->init(1/control_freq, tau,damping);
	
	auto accel_low_pass_filter = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	accel_low_pass_filter->init(1/control_freq, tau,damping);

	auto avel_low_pass_filter = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	avel_low_pass_filter->init(1/control_freq, tau,damping);

	auto aaccel_low_pass_filter = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	aaccel_low_pass_filter->init(1/control_freq, tau,damping);


	//robot state model
	Vector3d pos = Vector3d::Zero();
	Vector3d vel = Vector3d::Zero();
	Vector3d accel = Vector3d::Zero();
	Vector3d pos_local_model = Vector3d::Zero();
	Vector3d vel_local_model = Vector3d::Zero();
	Vector3d accel_local_model = Vector3d::Zero();

	Quaterniond orientation_aux;
	VectorXd orientaion_model = VectorXd::Zero(4);
	Vector3d avel = Vector3d::Zero();
	Vector3d aaccel = Vector3d::Zero();
	Vector3d avel_local_model = Vector3d::Zero();
	Vector3d aaccel_local_model = Vector3d::Zero();
	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();



		// read from Redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);
			

		// update robot model
		if(flag_simulation)
		{	
			robot->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCELERATIONS_KEY);

			robot->updateModel();
			robot->coriolisForce(coriolis);
			accel_local = redis_client.getEigenMatrixJSON(LINEAR_ACC_KEY);
			avel_local = redis_client.getEigenMatrixJSON(ANGULAR_VEL_KEY);
			aaccel_local = redis_client.getEigenMatrixJSON(ANGULAR_ACC_KEY);
			g_local = redis_client.getEigenMatrixJSON(LOCAL_GRAVITY_KEY);
		}
		else
		{

			robot->updateKinematics();
			robot->rotation(R_link,link_name); 
			g_local = R_link.transpose()*robot->_world_gravity;
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
			accel_aux_imu = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			avel_aux_imu = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
			
			//adjust IMU data
			accel_aux_imu *= 9.81; //sensor output is in g
			//rotate in FT frame
			accel_aux = R_acc_in_ft*accel_aux_imu;
			avel_aux = R_acc_in_ft*avel_aux_imu;
			accel_aux +=g_local; 

			// Kalman filter
			robot->position(current_position_aux, link_name, pos_in_link);
			current_position = R_link.transpose()*current_position_aux;
			y << current_position_aux, accel_aux;
			kalman_filter->update(y);
			kf_states = kalman_filter->state();
			accel_kf << kf_states(6), kf_states(7), kf_states(8);

			//Extended Kalman Filter
			q_eff = R_link; // in Eigen is vector part first -> needs to be adjusted
			q_eff_aux << q_eff.w(), q_eff.vec();			
			y_ekf << q_eff_aux, avel_aux;
			extended_kalman_filter-> update(y_ekf);
			ekf_states = extended_kalman_filter->state();
			avel_kf << ekf_states(4), ekf_states(5), ekf_states(6);
			aaccel_kf << ekf_states(7), ekf_states(8), ekf_states(9);

			//LP fiter
			accel_lp_filtered = accel_low_pass_filter->process(accel_aux);
      		avel_lp_filtered = avel_low_pass_filter->process(avel_aux);
            aaccel_lp = avel_low_pass_filter->getDerivative();
            aaccel_lp_filtered = aaccel_low_pass_filter->process(aaccel_lp);
            force_moment_lp_filtered = force_moment_low_pass_filter->process(force_moment);
        }
		//logging kinematics
        robot->position(pos, link_name, pos_in_link);
        robot->linearVelocity(vel, link_name, pos_in_link);
        robot->linearAcceleration(accel, link_name, pos_in_link);
        robot->angularVelocity(avel, link_name);
        robot->angularAcceleration(aaccel, link_name);
        
        pos_local_model = R_link.transpose()*pos;
        vel_local_model = R_link.transpose()*pos;
        orientation_aux = R_link;
        orientaion_model << orientation_aux.w(), orientation_aux.vec();
        avel_local_model = R_link.transpose()*avel;
        aaccel_local_model = R_link.transpose()*aaccel;

 	 	if(state == GOTO_INITIAL_CONFIG)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			cout << "command_torques: " << command_torques.transpose() << endl;
			if(config_error.norm() < 0.05)
			{
				joint_trajectory->init(desired_initial_configuration_trunc);
			    state = SINUSOIDAL;					
			}
		}
		else if(state == SINUSOIDAL)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_trajectory->update(trajectory_counter);
			q_des = joint_trajectory->getJointAngles();
			cout << "q_des: " << q_des << " q_robot: " <<  robot->_q.tail(axis).transpose() << endl;

			if(trajectory_counter <=5000)
			{
				dq_des = trajectory_counter/5000 *joint_trajectory->getJointVelocities();
				ddq_des = trajectory_counter/5000*joint_trajectory->getJointAccelerations();
			}
			else
			{
				dq_des = joint_trajectory->getJointVelocities();
				ddq_des = joint_trajectory->getJointAccelerations();
			}
			
			cout << "dq_des: " << dq_des.transpose() << " dq_robot: " <<  robot->_dq.tail(axis).transpose() << endl;

			cout << "ddq_des: " << ddq_des.transpose() << " ddq_robot: " <<  robot->_ddq.tail(axis).transpose() << endl;


			joint_task->_goal_position.tail(axis) = q_des;
			joint_task->_desired_velocity.tail(axis)= dq_des;

			if(controller_counter % 2 == 0)
			{

				if(flag_simulation)
				{
					RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
					phi_RLS = RLS->getInertialParameterVector();
				}
				else
				{
					RLS_KF->addData(force_moment_lp_filtered, accel_kf, avel_kf, aaccel_kf, g_local);
					RLS_LP->addData(force_moment_lp_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);

					phi_RLS_LP = RLS_LP->getInertialParameterVector();
					phi_RLS_KF = RLS_KF->getInertialParameterVector();
				}

			}
		

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			
			trajectory_counter++;

			if ((trajectory_counter/w_s) >= trajectory_counter_multiple *(2*M_PI/w_f) )
			{
				cout << "excictation period finished" << endl;

				trajectory_counter_multiple ++; 
				if (trajectory_counter_multiple == 1)
				{
					state = REST;
				}

			}



		}

		else if(state == REST)
		{

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques   + coriolis;

		}


		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(!flag_simulation)
		{
			if(logging_estimation_inputs)
			{
				redis_client.setEigenMatrixDerived(LINEAR_ACC_LP_KEY, accel_lp_filtered);
				redis_client.setEigenMatrixDerived(ANGULAR_VEL_LP_KEY, avel_lp_filtered);
				redis_client.setEigenMatrixDerived(ANGULAR_ACC_LP_KEY, aaccel_lp_filtered);
				redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
				redis_client.setEigenMatrixDerived(LINEAR_ACC_KF_KEY, accel_kf);
				redis_client.setEigenMatrixDerived(ANGULAR_VEL_KF_KEY, avel_kf);
				redis_client.setEigenMatrixDerived(ANGULAR_ACC_KF_KEY, aaccel_kf);
			}
			if(logging_estimation_outputs)
			{
				redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_LP_KEY,phi_RLS_LP);
				redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KF_KEY,phi_RLS_KF);	
			}
			if(logging_model_kinematics)
			{
				redis_client.setEigenMatrixDerived(POSITION_KIN_KEY, pos_local_model);
				redis_client.setEigenMatrixDerived(LINEAR_VEL_KIN_KEY, vel_local_model);
				redis_client.setEigenMatrixDerived(LINEAR_ACC_KIN_KEY, accel_local_model);
				redis_client.setEigenMatrixDerived(ORIENTATION_KIN_KEY, orientaion_model);
				redis_client.setEigenMatrixDerived(ANGULAR_VEL_KIN_KEY, avel_local);
				redis_client.setEigenMatrixDerived(ANGULAR_ACC_KIN_KEY, aaccel_local);

			}
		}
		if(logging_joint_trajectory)
		{
			redis_client.setEigenMatrixDerived(JOINT_ANGLE_INPUTS_KEY, q_des);
			redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_INPUTS_KEY, dq_des);
			redis_client.setEigenMatrixDerived(JOINT_ACCELERATIONS_INPUTS_KEY, ddq_des);
		}

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);


    double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Loop run time  : " << end_time << " seconds\n";
    cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;

}
