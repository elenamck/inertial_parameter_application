#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
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

const string robot_file = "../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

const bool inertia_regularization = true;
// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string LOCAL_GRAVITY_KEY;

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

std::string LINEAR_VEL_KF_KEY;

// - inertial parameters
std::string INERTIAL_PARAMS_KEY;
std::string INERTIAL_PARAMS_LS_KEY;
std::string INERTIAL_PARAMS_DEBUG_KEY; //computed with functions, not the library

// - kinematics:
std::string POSITION_KEY;
std::string LINEAR_VEL_KIN_KEY;
std::string LINEAR_ACC_KIN_KEY;
std::string ORIENTATION_QUATERNION_KEY;
std::string ANGULAR_VEL_KIN_KEY;
std::string ANGULAR_ACC_KIN_KEY;

#define  GOTO_INITIAL_CONFIG 	 0
#define	 MOVE_FIRST			     1
#define  MOVE_SECOND	         2
#define  MOVE_THIRD				 3
#define  MOVE_FORTH 			 4  



int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";
		QUATERNION_KEY = "sai2::DemoApplication::simulation::Panda::controller::logging::quaternion";
		

		//InertialParameters
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";
		INERTIAL_PARAMS_LS_KEY = "sai2::DemoApplication::Panda::controller::phiLS";
		INERTIAL_PARAMS_DEBUG_KEY = "sai2::DemoApplication::Panda::controller::phidebug";

		//Kinematics
		POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
		LINEAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
		LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::accel";
		ORIENTATION_QUATERNION_KEY =  "sai2::DemoApplication::Panda::kinematics::ori::quats";
		ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::avel";
		ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::aaccel";

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
		QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
		POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";

		LINEAR_VEL_KIN_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
		LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel";
		ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::sensors::avel";
		ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";

		LINEAR_VEL_KF_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
		
	}

	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	ifstream bias;
	bias.open("FT_data1.txt");
	if (!bias)  
	{                     // if it does not work
        cout << "Can't open Data!" << endl;
    }
    else
    {
    	
    	for (int row=0 ; row<6; row++)
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

	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	robot->updateModel();


	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);



	//Controllers
	Vector3d vel_sat = Vector3d(0.3,0.3,0.3);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.15);
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	pos_task->_max_velocity = 0.11;

	pos_task->_kp = 100.0;
	pos_task->_kv = 2.1*sqrt(pos_task->_kp);

	pos_task->_velocity_saturation = true;
	pos_task->_saturation_velocity = vel_sat;
	VectorXd pos_task_torques = VectorXd::Zero(dof);

	Vector3d pos_des_1 = Vector3d(.5,0.4,0.1);
	Vector3d pos_des_2 = Vector3d(.5,-0.4,0.6);
	Vector3d pos_des_3 = Vector3d(.5,-0.4,0.1);
	Vector3d pos_des_4 = Vector3d(.5,0.4,0.6);

	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/6.0;
	joint_task->_kp = 50.0;
	joint_task->_kv = 2.4 * sqrt(joint_task->_kp);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;
	// desired_initial_configuration << 0, 10, 0, -125, 0, 135, 0;


	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;


	Matrix3d current_orientation = Matrix3d::Zero();



	Matrix3d R_link = Matrix3d::Zero();

	Vector3d kf_vel = Vector3d::Zero();
	Vector3d accel_kin = Vector3d::Zero();

	//For Inertial Parameter Estimation
	bool linear_case = true;
	bool non_linear_case = false;
	Matrix3d Lambda_lin = 0.01*Matrix3d::Identity();
	MatrixXd Lambda = 0.014 * MatrixXd::Identity(6,6);

	// auto RLS = new ParameterEstimation::RecursiveLeastSquare(linear_case,4,Lambda_lin);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,7,Lambda);


	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	Vector3d force_sensed = Vector3d::Zero();
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();



	//position and orientation
	Vector3d position =  Vector3d::Zero();
	Vector3d velocity =  Vector3d::Zero();
	Matrix3d orientation = Matrix3d::Zero();
	Quaterniond orientation_quaternion;
	VectorXd orientation_quaternion_aux = VectorXd::Zero(4);


	int state = GOTO_INITIAL_CONFIG;

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
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
		force_sensed << force_moment(0), force_moment(1), force_moment(2);	
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


			orientation = R_link.transpose();
			orientation_quaternion = orientation;
			orientation_quaternion.normalize();
			orientation_quaternion_aux << orientation_quaternion.w() , orientation_quaternion.vec();
			robot->position(position, link_name, pos_in_link);
			position = R_link.transpose()*position;
			robot->linearVelocity(velocity, link_name);
			velocity = R_link.transpose()*velocity;

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

		}


		RLS_2 ->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
		phi_RLS = RLS_2->getInertialParameterVector();


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
				joint_task->reInitializeTask();

				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";

			    pos_task->_goal_position = pos_des_1;
			    joint_task->_goal_position(6) -= 2.0*M_PI;

			    state = MOVE_THIRD;
				
			}
		}



		else if(state == MOVE_FIRST)
		{

			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques  + pos_task_torques + coriolis;

			VectorXd config_error_pos = pos_task->_goal_position - pos_task->_current_position;
			if(config_error_pos.norm() < 0.2)
			{	
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				pos_task->enableVelocitySaturation(vel_sat);

				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
			
				pos_task->_goal_position = pos_des_2;
			    joint_task->_goal_position(6) += 2.0*M_PI;

				state = MOVE_SECOND;		
			}


		}

		
		else if(state == MOVE_SECOND)
		{

			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques  + pos_task_torques + coriolis;

			VectorXd config_error_pos = pos_task->_goal_position - pos_task->_current_position;
			if(config_error_pos.norm() < 0.2)
			{	
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				pos_task->enableVelocitySaturation(vel_sat);

				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
			
				pos_task->_goal_position = pos_des_3;
			    joint_task->_goal_position(6) -= 2.0*M_PI;

				state = MOVE_THIRD;		
			}


		}
		else if(state == MOVE_THIRD)
		{

			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques  + pos_task_torques + coriolis;

			VectorXd config_error_pos = pos_task->_goal_position - pos_task->_current_position;
			if(config_error_pos.norm() < 0.2)
			{	
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				pos_task->enableVelocitySaturation(vel_sat);

				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
			
				pos_task->_goal_position = pos_des_4;
			    joint_task->_goal_position(6) += 2.0*M_PI;

				state = MOVE_FORTH;		
			}


		}
		else if(state == MOVE_FORTH)
		{

			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques  + pos_task_torques + coriolis;


		}

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);

		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS);


		redis_client.setEigenMatrixDerived(POSITION_KEY, position);
		redis_client.setEigenMatrixDerived(LINEAR_VEL_KIN_KEY, velocity);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KIN_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ORIENTATION_QUATERNION_KEY, orientation_quaternion_aux);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KIN_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KIN_KEY, aaccel_local);

		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, force_moment);


		

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
	inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
	std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";

    return 0;

}






