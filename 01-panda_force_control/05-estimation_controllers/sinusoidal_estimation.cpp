#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "trajectories/JointSpaceSinusodial.h"
#include "filters/MeanFilter.h"
#include "filters/ButterworthFilter.h"
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

const string robot_file = "../../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;
const bool logging_kinematics = false;

const bool inertia_regularization = true;
// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string LINEAR_ACC_KEY;
std::string ANGULAR_VEL_KEY;
std::string ANGULAR_ACC_KEY;
std::string LOCAL_GRAVITY_KEY;
std::string INERTIAL_PARAMS_KEY;
std::string EE_FORCE_SENSOR_UNBIASED_KEY;
std::string POSITION_KEY;
std::string LINEAR_VEL_KEY;



// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_ACCELERATIONS_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;
std::string ACCELEROMETER_DATA_KEY;
std::string GYROSCOPE_DATA_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

string JOINT_ANGLE_INPUTS_KEY;
string JOINT_VELOCITIES_INPUTS_KEY;
string JOINT_ACCELERATIONS_INPUTS_KEY;

std::string POSITION_GLOBAL_KEY;
std::string LINEAR_VEL_GLOBAL_KEY;
std::string LINEAR_ACC_GLOBAL_KEY;
std::string ANGULAR_VEL_GLOBAL_KEY;
std::string ANGULAR_ACC_GLOBAL_KEY;

string 	LINEAR_ACC_LP_KEY;
string	ANGULAR_VEL_LP_KEY;
string	ANGULAR_ACC_LP_KEY;

#define  GOTO_INITIAL_CONFIG 	 0
#define  SINUSODIAL				 1
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
		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::Panda::simulation::g_local";
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";

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

		//corrected sensor data(accelerometer: gravity removed, right frame, Gyroscope: right frame)
		LINEAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel";
		LOCAL_GRAVITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::g_local";

		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::controller::phi";

		JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::FrankaPanda::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::FrankaPanda::desired::dq";

		LINEAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel::lowpass";
		ANGULAR_VEL_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel::lowpass";
		ANGULAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel::lowpass";




		EE_FORCE_SENSOR_UNBIASED_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";	
		if(logging_kinematics)
		{	

			POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";
			LINEAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::controller::vel";
			POSITION_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos_global";
 			LINEAR_VEL_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::vel_global";
			LINEAR_ACC_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel_global";
			ANGULAR_VEL_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel_global";
			ANGULAR_ACC_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel_global"; 
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
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.107);



	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);

	joint_task->_max_velocity = M_PI/6;
	joint_task->_kp = 100.0;
	joint_task->_kv = 1.6 * sqrt(joint_task->_kp);


	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	//For Inertial Parameter Estimation

	bool non_linear_case = false;
	// MatrixXd Lambda = 0.05 * MatrixXd::Identity(6,6);
	MatrixXd Lambda = 0.07 * MatrixXd::Identity(6,6);
	MatrixXd Lambda_LP = 0.1 * MatrixXd::Identity(6,6);

	auto RLS = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,6, Lambda);
	auto RLS_LP = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,6, Lambda_LP);
	// auto LS = new ParameterEstimation::LeastSquare(non_linear_case);


	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	VectorXd phi_RLS_LP = VectorXd::Zero(10);
	Vector3d force_sensed = Vector3d::Zero();
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();
	Matrix3d inertia_tensor_RLS_LP = Matrix3d::Zero();
	Vector3d center_of_mass_RLS_LP = Vector3d::Zero();

	Vector3d pos = Vector3d::Zero();
	Vector3d vel = Vector3d::Zero();
	Vector3d pos_local = Vector3d::Zero();
	Vector3d vel_local = Vector3d::Zero();


	MatrixXd A_full = MatrixXd::Zero(6,10);



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

    // Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4; -------------------TODO check which are better!--------------------------
    // R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-3, 1.0e-3, 1.0e-3;
    Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4;

    // Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6;
    R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-2, 1.0e-2, 1.0e-2;

    auto kalman_filter = new KalmanFilters::KalmanFilter(dt, A, C, Q, R, P);


    VectorXd x0 = VectorXd::Zero(n_kf);
    double t0 = 0;
    kalman_filter->init(t0, x0);
    VectorXd y = VectorXd::Zero(m_kf);
    VectorXd kf_states = VectorXd::Zero(n_kf);
    Vector3d accel_aux = Vector3d::Zero();
    Vector3d current_position_aux = Vector3d::Zero();

	

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
  	Vector3d avel_aux = Vector3d::Zero();

    //Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;
	Matrix3d R_link = Matrix3d::Zero();


	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_moment_filtered = VectorXd::Zero(6);
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
    cout << "bias read" << force_torque_bias << endl;
    bias.close();
	}
	//for sinusodial trajectories
	int axis = 4;
	int N = 3;
	double w_s = control_freq;
	double w_f = 0.8; 
	VectorXd q_des = VectorXd::Zero(axis);
	VectorXd dq_des = VectorXd::Zero(axis);
	VectorXd ddq_des = VectorXd::Zero(axis);

	VectorXd a = VectorXd::Zero(N*axis);
	VectorXd b = VectorXd::Zero(N*axis);
	// a << -0.0286409,         0.0322926,       0.47785,         -0.571294,       0.0973072,       -0.10507,        -0.194213,       -0.327815,       0.261298,        -0.659976,       0.634429,        0.0897043;
	a<< 0.259086 ,-0.00621783   , 0.429696 ,  -0.728262,   -0.780967  ,  -0.23069 ,  -0.178586 ,   0.267967 ,  -0.723327  ,  0.641493  , -0.304355  , -0.505646;


	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	VectorXd desired_initial_configuration_trunc = VectorXd::Zero(axis);
	desired_initial_configuration_trunc = desired_initial_configuration.tail(axis);
	joint_trajectory->init(desired_initial_configuration_trunc);
	// cout << "desired_initial_configuration_trunc " << desired_initial_configuration_trunc << endl;
	unsigned int long trajectory_counter = 0;
	int trajectory_counter_multiple = 1;
	joint_trajectory->update(trajectory_counter);
	joint_task->_desired_velocity.tail(axis) = joint_trajectory->getJointVelocities();

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

	//cutoff frequeny, try frequency of trajectory
	double fc = 0.6;
	//normalized cutoff frequeency, for butterworth
	double fc_n = fc / (0.5*control_freq);

	//time constant, for lowpassfilter
	double tau = 1/(2*M_PI*0.7);

	double fc_f = 0.6; 
	double fc_t = 0.6; 



	// auto force_filter = new ButterworthFilter(3);
	// force_filter->setCutoffFrequency(fc_n);
	auto force_moment_filter = new ButterworthFilter(6);
	force_moment_filter->setCutoffFrequency(fc_f/(0.5*control_freq));

	auto accel_butter_filter = new ButterworthFilter(3);
	accel_butter_filter->setCutoffFrequency(fc_n);

	auto avel_butter_filter = new ButterworthFilter(3);
	avel_butter_filter->setCutoffFrequency(fc_n);

	auto accel_low_pass_filter = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	accel_low_pass_filter->init(1/control_freq, tau,0.7);
	Vector3d accel_lp_filtered = Vector3d::Zero();

	auto avel_low_pass_filter = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	avel_low_pass_filter->init(1/control_freq, tau,0.8);
	Vector3d avel_lp_filtered = Vector3d::Zero();
	Vector3d aaccel_lp_filtered = Vector3d::Zero();




	// create timer
	std::chrono::high_resolution_clock::time_point t_start;
	std::chrono::duration<double> t_elapsed;
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
			force_moment_filtered =  force_moment_filter->update(force_moment);

			// robot->position(current_position_aux, link_name, pos_in_link);
			// current_position_aux = R_link.transpose()*current_position_aux;
			accel_aux = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			avel_aux = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);

			accel_aux = R_acc_in_ft*accel_aux;
			accel_aux *= 9.81;
			accel_aux += g_local;
            avel_aux = R_acc_in_ft*avel_aux;

			accel_local = accel_butter_filter->update(accel_aux);
			accel_lp_filtered = accel_low_pass_filter->process(accel_aux);
			// y << current_position_aux, accel_aux;
   //          kalman_filter->update(y);
   //          kf_states = kalman_filter->state();
   //          accel_local << kf_states(6), kf_states(7), kf_states(8);

   //          avel_aux *= M_PI/180;
            avel_local = avel_butter_filter->update(avel_aux);
            avel_lp_filtered = avel_low_pass_filter->process(avel_aux);
            aaccel_lp_filtered = avel_low_pass_filter->getDerivative();
   //          q_eff = R_link;
			// q_eff_aux << q_eff.w(), q_eff.vec();

			// y_ekf << q_eff_aux, avel_aux;
			// extended_kalman_filter-> update(y_ekf);
			// ekf_states = extended_kalman_filter->state();
			// avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
			// aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);
			//----------------------------------------------------------

		// robot->linearAcceleration(accel,link_name, pos_in_link);
		robot->angularAcceleration(aaccel,link_name);
		// robot->angularVelocity(avel,link_name);
		// //get object acc in sensor frame
		// accel_local = R_link.transpose()*accel;
		aaccel_local = R_link.transpose()*aaccel;
		// cout << "aaccel_local" << aaccel_local.transpose() << endl;
		//get object velocity in sensor frame
		// avel_local = R_link.transpose()*avel;

		if(logging_kinematics)
		{
			robot->position(pos,link_name, pos_in_link);
			robot->linearVelocity(vel,link_name, pos_in_link);
			pos_local = R_link.transpose() * pos;
			vel_local = R_link.transpose() * vel;
		}

		}
		t_start = std::chrono::high_resolution_clock::now();

		t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;
		//cout << "Elapsed time Inertial Parameter Estimation: " << t_elapsed.count() << endl;

 	 	if(state == GOTO_INITIAL_CONFIG)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.1)
			{

				joint_trajectory->init(desired_initial_configuration_trunc);

				
					cout << "Initial Config reached" << endl;
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
			// cout << "q_des: " << q_des << " q_robot: " <<  robot->_q(6) << endl;

			dq_des = joint_trajectory->getJointVelocities();
						// cout << "dq_des: " << dq_des << " dq_robot: " <<  robot->_dq(6) << endl;

			ddq_des = joint_trajectory->getJointAccelerations();
						// cout << "ddq_des: " << ddq_des << " ddq_robot: " <<  robot->_ddq(6) << endl;


			joint_task->_goal_position.tail(axis) = q_des;
			joint_task->_desired_velocity.tail(axis)= dq_des;

			// mean_accel->process(accel_local_mean, accel_local);
			// mean_avel->process(avel_local_mean, avel_local);
			// mean_aaccel->process(aaccel_local_mean, aaccel_local);
			// mean_g_local->process(g_local_mean, g_local);
			// mean_force_moment->process(force_moment_mean, force_moment);


			if (controller_counter % 2 == 0 )
			{	
			mean_accel->process(accel_local_mean, accel_local);
			mean_avel->process(avel_local_mean, avel_local);
			mean_aaccel->process(aaccel_local_mean, aaccel_local);
			mean_g_local->process(g_local_mean, g_local);
			mean_force_moment->process(force_moment_mean, force_moment);
			}
			if(controller_counter % 2 == 0)
			{
				// RLS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
				// RLS->addData(force_moment_filtered, accel_local, avel_local, aaccel_lp_filtered, g_local);

				if(flag_simulation)
				{
					RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
				}
				else{
									RLS->addData(force_moment_filtered, accel_local, avel_local, aaccel_lp_filtered, g_local);
				RLS_LP->addData(force_moment_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);
				}

				// LS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
				phi_RLS = RLS->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);

				phi_RLS_LP = RLS_LP->getInertialParameterVector();
				center_of_mass_RLS_LP << phi_RLS_LP(1)/phi_RLS_LP(0), phi_RLS_LP(2)/phi_RLS_LP(0), phi_RLS_LP(3)/phi_RLS_LP(0); 
				inertia_tensor_RLS_LP << phi_RLS_LP(4), phi_RLS_LP(5), phi_RLS_LP(6), phi_RLS_LP(5), phi_RLS_LP(7), phi_RLS_LP(8), phi_RLS_LP(6), phi_RLS_LP(8), phi_RLS_LP(9);


			}
		
			// RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
			// phi_RLS = RLS->getInertialParameterVector();
			// center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			// inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			// // t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;
			// cout << "Elapsed time trajectory update: " << t_elapsed.count() << endl;

			if(controller_counter%1000==0)
			{
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS << endl;

		   		cout << "estimated mass LP: \n" << phi_RLS_LP(0) << endl;
		  	  	cout << "estimated center of mass LP: \n" << 	center_of_mass_RLS_LP.transpose() << endl;
		   		cout << "estimated Inertia LP: \n" << inertia_tensor_RLS_LP << endl;
			}

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			
			trajectory_counter++;

			if ((trajectory_counter/w_s) >= trajectory_counter_multiple *(2*M_PI/w_f) )
			{
				cout << "excictation period finished" << endl;

				// phi_LS = LS->getInertialParameterVector();
				// center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
				// inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				// cout << "estimated mass LS: \n" << phi_LS(0) << endl;
		  // 	  	cout << "estimated center of mass LS: \n" << 	center_of_mass_LS.transpose() << endl;
		  //  		cout << "estimated Inertia LS: \n" << inertia_tensor_LS << endl;

				// state = REST;
				trajectory_counter_multiple ++; 
				// RLS->init();
				if (trajectory_counter_multiple == 3)
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
		if(logging_kinematics)
		{	
			redis_client.setEigenMatrixDerived(POSITION_KEY, pos_local);
			redis_client.setEigenMatrixDerived(LINEAR_VEL_KEY, vel_local);
			// redis_client.setEigenMatrixDerived(POSITION_GLOBAL_KEY, pos);
			// redis_client.setEigenMatrixDerived(LINEAR_VEL_GLOBAL_KEY, vel);
			// redis_client.setEigenMatrixDerived(LINEAR_ACC_GLOBAL_KEY, accel);
			// redis_client.setEigenMatrixDerived(ANGULAR_VEL_GLOBAL_KEY, avel);
			// redis_client.setEigenMatrixDerived(ANGULAR_ACC_GLOBAL_KEY, aaccel);

		}
		if(!flag_simulation)
		{
			redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
			redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
			redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
			redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
			redis_client.setEigenMatrixDerived(LINEAR_ACC_LP_KEY, accel_lp_filtered);
			redis_client.setEigenMatrixDerived(ANGULAR_VEL_LP_KEY, avel_lp_filtered);
			redis_client.setEigenMatrixDerived(ANGULAR_ACC_LP_KEY, aaccel_lp_filtered);
		}


		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_UNBIASED_KEY, force_moment);

		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS);
		redis_client.setEigenMatrixDerived(JOINT_ANGLE_INPUTS_KEY, q_des);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_INPUTS_KEY, dq_des);
		redis_client.setEigenMatrixDerived(JOINT_ACCELERATIONS_INPUTS_KEY, ddq_des);




		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);


    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;

}
