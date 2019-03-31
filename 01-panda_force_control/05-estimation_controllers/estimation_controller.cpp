#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"


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

const bool inertia_regularization = true;
// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string LINEAR_ACC_KEY;
std::string ANGULAR_VEL_KEY;
std::string ANGULAR_ACC_KEY;
std::string LOCAL_GRAVITY_KEY;
std::string INERTIAL_PARAMS_KEY;

// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;
std::string ACCELEROMETER_DATA_KEY;
std::string GYROSCOPE_DATA_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// - offline processing
std::string LINEAR_ACCELERATION_LOCAL_KEY;
std::string ANGULAR_VELOCITY_LOCAL_KEY;
std::string EE_FORCE_SENSOR_KEY;
std::string QUATERNION_KEY;
std::string POSITION_KEY;


std::string LINEAR_ACC_KIN_KEY;

#define  GOTO_INITIAL_CONFIG 	 0
#define  MOVE_TO_OBJECT          1
#define  MOVE_UP 				 2
#define  MOVE_DOWN 				 3  
#define  MOVE 					 4
#define  HAND_OVER 				 5


int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

		LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::accel";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";
		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";

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
		LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
		ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
		EE_FORCE_SENSOR_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";
		QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
		POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";
		
	}

	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	ifstream bias;
	bias.open("../../02-utilities/FT_data1.txt");
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
    cout << "test" << endl;			
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
	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	Vector3d avel_sat = Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.1);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.11;

	posori_task->_kp_pos = 60.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_ori);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// position controller for angular motion

	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task2->_max_velocity = 0.1;

	posori_task2->_kp_pos = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_pos);
	posori_task2->_kp_ori = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_ori);
	posori_task2->_velocity_saturation = true;
	posori_task2->_linear_saturation_velocity = vel_sat;
	posori_task2->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task2_torques = VectorXd::Zero(dof);
	Vector3d current_position = Vector3d::Zero();

	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/7.5;
	joint_task->_kp = 41.0;
	joint_task->_kv = 2.4 * sqrt(joint_task->_kp);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;
	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	//For Inertial Parameter Estimation

	bool linear_case = true;
	bool non_linear_case = false;
	Matrix3d Lambda_lin = 0.01*Matrix3d::Identity();
	MatrixXd Lambda = 0.01 * MatrixXd::Identity(6,6);
	auto LS = new ParameterEstimation::LeastSquare(linear_case);
	auto LS_2 = new ParameterEstimation::LeastSquare(non_linear_case);

	auto RLS = new ParameterEstimation::RecursiveLeastSquare(linear_case,4,Lambda_lin);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,4,Lambda);


	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd phi_LS = VectorXd::Zero(10); //inertial parameter vector
	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	VectorXd phi_lin_LS = VectorXd::Zero(4); //inertial parameter vector
	VectorXd phi_lin_RLS = VectorXd::Zero(4); //inertial parameter vector
	Vector3d force_sensed = Vector3d::Zero();
	Matrix3d inertia_tensor_LS = Matrix3d::Zero();
	Vector3d center_of_mass_LS = Vector3d::Zero();
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();

	MatrixXd A_lin = MatrixXd::Zero(3,4);
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

    Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6;
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
			
		robot->rotation(R_link,link_name); 
		g_local = R_link.transpose()*robot->_world_gravity;
		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
			robot->linearAcceleration(accel,link_name, Eigen::Vector3d::Zero());
			robot->angularAcceleration(aaccel,link_name);
			robot->angularVelocity(avel,link_name);
			accel_local = R_link.transpose()*accel;
			accel_local += g_local;
			aaccel_local = R_link.transpose()*aaccel;
			avel_local = R_link.transpose()*avel;		 
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

			robot->position(current_position_aux, link_name, Vector3d::Zero());
			current_position_aux = R_link.transpose()*current_position_aux;
			accel_aux = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			accel_aux = R_acc_in_ft*accel_aux;
			accel_aux *= 9.81;
			accel_aux += g_local;
			y << current_position_aux, accel_aux;
            kalman_filter->update(y);
            kf_states = kalman_filter->state();
            accel_local << kf_states(6), kf_states(7), kf_states(8);

            avel_aux = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
            avel_aux = R_acc_in_ft*avel_aux;
            avel_aux *= M_PI/180;

            q_eff = R_link.transpose();
			q_eff_aux << q_eff.w(), q_eff.vec();

			y_ekf << q_eff_aux, avel_aux;
			extended_kalman_filter-> update(y_ekf);
			ekf_states = extended_kalman_filter->state();
			avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
			aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);


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
			if(config_error.norm() < 0.26)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE_TO_OBJECT;
				
			}

		}
		if(state == MOVE_TO_OBJECT)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			posori_task2->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			Vector3d desired_position = Vector3d(.4,0.3,0.4);

			posori_task->_goal_position = desired_position;


			// compute torques
			posori_task->computeTorques(posori_task_torques);
			posori_task2->computeTorques(posori_task2_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task2->reInitializeTask();
					posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
					state = MOVE_UP;				
			}

		}

		if(state == MOVE_UP)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,0.3,0.9);

			// compute torques

			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;



			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE_DOWN;
				
			}

		}


		if(state == MOVE_DOWN)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,0.3,0.4);



			// compute torques

			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;



			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE;
				
			}

		}

		if(state == MOVE)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			posori_task2->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,-0.5,0.4);

			robot->position(current_position, posori_task->_link_name, pos_in_link);
			double circle_radius = 0.3;
			double circle_freq = 0.3;
			posori_task2->_desired_position = current_position + circle_radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()));
			posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, 0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()));

			// compute torques

			posori_task2->computeTorques(posori_task2_torques);
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis + posori_task2_torques;


			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				joint_task->_goal_position(5) += M_PI/2;
				state = HAND_OVER;
				
			}

		}


		if(state == HAND_OVER)
		{

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			
			N_prec = joint_task->_N_prec;
			posori_task->updateTaskModel(N_prec);

			


			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;


		}



		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);


		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS);




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
