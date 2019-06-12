#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "trajectories/JointSpaceSinusodial.h"


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

		JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";
		JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::ddq";

		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";

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
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
	if (flag_simulation)
	{
		redis_client.getEigenMatrixDerived(JOINT_ACCELERATIONS_KEY, robot->_ddq);

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

	joint_task->_max_velocity = M_PI/4;
	joint_task->_kp = 100.0;
	joint_task->_kv = 2.0 * sqrt(joint_task->_kp);


	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	//For Inertial Parameter Estimation

	MatrixXd Lambda = 0.05 * MatrixXd::Identity(6,6);

	auto RLS = new ParameterEstimation::RecursiveLeastSquare(6, Lambda);
	auto LS = new ParameterEstimation::LeastSquare();



	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();
	VectorXd phi_LS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_LS = Matrix3d::Zero();
	Vector3d center_of_mass_LS = Vector3d::Zero();


	if (flag_simulation)
	{	

		redis_client.getEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.getEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.getEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.getEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);		
	}





	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);




	//for sinusodial trajectories
	int axis = 4;
	int N = 3;
	// double w_s = control_freq;
	double w_s = 700;
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


	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();


		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		

		// update robot model
		if(flag_simulation)
		{	
			redis_client.getEigenMatrixDerived(JOINT_ACCELERATIONS_KEY, robot->_ddq);

			robot->updateModel();
			robot->coriolisForce(coriolis);
			redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY,force_moment);	
			redis_client.getEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
			redis_client.getEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
			redis_client.getEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
			redis_client.getEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);	
		}
		else
		{

		}


 	 	if(state == GOTO_INITIAL_CONFIG)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.02)
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


			dq_des = joint_trajectory->getJointVelocities();


			ddq_des = joint_trajectory->getJointAccelerations();


			joint_task->_goal_position.tail(axis) = q_des;
			joint_task->_desired_velocity.tail(axis)= dq_des;



			if(controller_counter % 2 == 0)
			{

				RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
				LS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
				phi_RLS = RLS->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			}



			if(controller_counter%1000==0)
			{
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS << endl;

		   			
				// cout << "accel_local: " << accel_local.transpose() << endl;
				// cout << "avel_local: " << avel_local.transpose() << endl;
				// cout << "aaccel_local: " << aaccel_local.transpose() << endl;
				// cout << "g_local: " << g_local.transpose() << endl;
				// cout << "force_moment: " << force_moment.transpose() << endl;
			}

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			
			trajectory_counter++;

			if ((trajectory_counter/w_s) >= trajectory_counter_multiple *(2*M_PI/w_f) )
			{
				cout << "excictation period finished" << endl;
				phi_LS = LS->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				cout << "estimated mass LS: \n" << phi_LS(0) << endl;
		  	  	cout << "estimated center of mass LS: \n" << 	center_of_mass_LS.transpose() << endl;
		   		cout << "estimated Inertia LS: \n" << inertia_tensor_LS << endl;

				state = REST;
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
