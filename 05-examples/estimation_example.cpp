#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>
#include "timer/LoopTimer.h"
#include "filters/SecOrderLowPass.hpp"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "Sai2Primitives.h"

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "FRANKA-PANDA";

#define  GOTO_INITIAL_CONFIG 0
#define  MOVE_1        		 1
#define  MOVE_2       		 2
#define  MOVE_3       		 3
#define  REST 				 4

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

const bool inertia_regularization = true;
// redis keys:
// - write:
string JOINT_TORQUES_COMMANDED_KEY;
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_ACCELERATIONS_KEY;
string ACCELEROMETER_DATA_KEY;
string GYROSCOPE_DATA_KEY;
string EE_FORCE_SENSOR_FORCE_KEY;
// estimation inputs from simulated robot
string LINEAR_ACC_KEY;
string ANGULAR_VEL_KEY;
string ANGULAR_ACC_KEY;
string LOCAL_GRAVITY_KEY;

// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;



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
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";	
		ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";      
		GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";
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
	robot->updateModel();
	// read from Redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

	if(flag_simulation)
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

	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.2);

	//IMU sensor data
	Vector3d accel_aux_imu = Vector3d::Zero(); //sensor data, IMU frame
	Vector3d avel_aux_imu = Vector3d::Zero(); //sensor data, IMU frame
	Vector3d accel_aux = Vector3d::Zero(); //sensor data FT frame
	Vector3d avel_aux = Vector3d::Zero(); //sensor data FT frame
	
	//low pass filter
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


	Vector3d accel_lp_filtered = Vector3d::Zero();
	Vector3d avel_lp_filtered = Vector3d::Zero();
	Vector3d aaccel_lp = Vector3d::Zero();
	Vector3d aaccel_lp_filtered = Vector3d::Zero();
	VectorXd force_moment_lp_filtered = VectorXd::Zero(6);

    //Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;
	Matrix3d R_link = Matrix3d::Zero();		//rotation base to end-effector

	//For Inertial Parameter Estimation
	//Measurement noise covariance matrix, values based on typical noise ft sensor
	MatrixXd Lambda = MatrixXd::Zero(6,6);
	Lambda   << 0.035, 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ,
				0.0  ,0.035, 0.0 , 0.0  , 0.0  , 0.0  ,
				0.0  , 0.0 , 0.15, 0.0  , 0.0  , 0.0  ,
				0.0  , 0.0 , 0.0 , 0.002, 0.0  , 0.0  ,
				0.0  , 0.0 , 0.0 , 0.0  , 0.002, 0.0  ,
				0.0  , 0.0 , 0.0 , 0.0  , 0.0  , 0.001;

	auto RLS = new ParameterEstimation::RecursiveLeastSquare(6, Lambda);

	//estimated inertial parameters
	VectorXd phi_RLS = VectorXd::Zero(10);


	//inertial parameter input
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd force_moment = VectorXd::Zero(6); 
	
	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	// pos ori controller

	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;

	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = 0.2;

	joint_task->_kp = 10.0;
	joint_task->_kv = 5.0;

	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;
	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

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
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
			accel_aux_imu = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			avel_aux_imu = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
			
			//adjust IMU data
			accel_aux_imu *= 9.81; //sensor output is in g
			//rotate in FT frame
			accel_aux = R_acc_in_ft*accel_aux_imu;
			avel_aux = R_acc_in_ft*avel_aux_imu;
			robot->rotation(R_link, link_name);
			accel_aux +=g_local; 



			//LP fiter
			accel_lp_filtered = accel_low_pass_filter->process(accel_aux);
      		avel_lp_filtered = avel_low_pass_filter->process(avel_aux);
            aaccel_lp = avel_low_pass_filter->getDerivative();
            aaccel_lp_filtered = aaccel_low_pass_filter->process(aaccel_lp);
            force_moment_lp_filtered = force_moment_low_pass_filter->process(force_moment);
			
			accel_local = accel_lp_filtered;
			avel_local = avel_lp_filtered;
			aaccel_local = aaccel_lp_filtered;
			force_moment = force_moment_lp_filtered;
		}
		if(state != GOTO_INITIAL_CONFIG)
		{
			RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
			phi_RLS = RLS->getInertialParameterVector();
			Vector3d center_of_mass;
			Matrix3d inertia_tensor;
			center_of_mass << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);


			if(controller_counter%1000==0)
			{
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor << endl;

			}

		}



				// state machine
		if(state == GOTO_INITIAL_CONFIG)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.05)
			{
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();
				Matrix3d desired_oriention;
				desired_oriention << 1, 0, 0,
									 0, 0, -1,
									 0, 1, 0;
				posori_task->_goal_position = Vector3d(0.5, -0.35, 0.4);
				posori_task->_desired_orientation = desired_oriention;

				state = MOVE_1;
			}

		}
		else if(state == MOVE_1)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;
			if(config_error.norm() < 0.05)
			{
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();
				Matrix3d desired_oriention;
				desired_oriention <<  1,  0,  0,
									  0,  -1, 0,
									  0,  0, -1;
				posori_task->_goal_position = Vector3d(0.6, 0.0, 0.55);
				posori_task->_desired_orientation = desired_oriention;

				state = MOVE_2;
			}

		}
		else if(state == MOVE_2)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;
			if(config_error.norm() < 0.05)
			{
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();
				Matrix3d desired_oriention;
				desired_oriention <<  1,  0, 0,
									  0,  0, 1,
									  0, -1, 0;
				posori_task->_goal_position = Vector3d(0.5, 0.35, 0.4);
				posori_task->_desired_orientation = desired_oriention;

				state = MOVE_3;
			}

		}
		else if(state == MOVE_3)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;
			if(config_error.norm() < 0.05)
			{
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();

				state = REST;
			}

		}
		else if(state == REST)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;

		}

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

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
// 