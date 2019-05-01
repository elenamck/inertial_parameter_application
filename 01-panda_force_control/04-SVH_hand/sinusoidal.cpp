#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"


#include "timer/LoopTimer.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "filters/SecOrderLowPass.hpp"
#include "filters/ButterworthFilter.h"
#include "trajectories/JointSpaceSinusodial.h"


#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>


#include <signal.h>
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
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_ACCELERATIONS_KEY;

string EE_FORCE_SENSOR_FORCE_KEY;

string LINEAR_ACC_KEY;
string ANGULAR_VEL_KEY;
string ANGULAR_ACC_KEY;
string LOCAL_GRAVITY_KEY;

string ACCELEROMETER_DATA_KEY;
string GYROSCOPE_DATA_KEY;
string SVH_HAND_POSITION_COMMAND_KEY;
string SVH_HAND_GRASP_FLAG_KEY;

// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;


#define  GOTO_INITIAL_CONFIG 	 	0
#define  MOVE_TO_CYLINDER          	1
#define  GRASP_CYLINDER 			2
#define  GRASP_CYLINDER_2 			8

#define  MOVE_ABOVE_WITH_OBJECT  	3

#define  MOVE_DOWN_WITH_OBJECT 	 	4
#define  LET_GO_OF_CYLINDER 		5
#define	 MOVE_BACK					6
#define	 REST						7

#define MOVE_CYLINDER_1 8
#define MOVE_CYLINDER_2 9
#define MOVE_CYLINDER_3 10
#define MOVE_CYLINDER_4 11


#define SINUSOIDAL 13
#define REST_SIN 12


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

		SVH_HAND_POSITION_COMMAND_KEY ="sai2::SVHHand_Left::position_command";
		SVH_HAND_GRASP_FLAG_KEY ="sai2::SVHHand_Left::grasp_type_flag";
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

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	const string ee_link = "link7";
	const Vector3d ee_pos_in_link = Vector3d(0.0, 0.0, 0.15);

	//Controllers
	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	Vector3d avel_sat = Vector3d(M_PI/5.5, M_PI/5.5, M_PI/5.5);
	// pos ori controller
	auto posori_task = new Sai2Primitives::PosOriTask(robot, ee_link, ee_pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 89.0;
	posori_task->_kv_pos = 2*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 78.0;
	posori_task->_kv_ori = 2.1*sqrt(posori_task->_kp_ori);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/8;
	joint_task->_kp = 100.0;
	joint_task->_kv = 2 * sqrt(joint_task->_kp);
	// joint_task->_ki = 4;
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	// desired_initial_configuration << -0.35,0,0,-1.5,0,1.5,1.6;
		desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	desired_initial_configuration *= M_PI/180.0;

	joint_task->_goal_position = desired_initial_configuration;

	//For Inertial Parameter Estimation
	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	Matrix3d R_link = Matrix3d::Zero();

	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();


	VectorXd phi_RLS_2 = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS_2 = Matrix3d::Zero();
	Vector3d center_of_mass_RLS_2 = Vector3d::Zero();

	double lambda_factor = 0.005;
	double lambda_factor_2 = 0.001;
	MatrixXd Lambda = lambda_factor*MatrixXd::Identity(6,6);

	int filter_size = 10;
	int filter_size_2 = 8;


	MatrixXd Lambda_2 = lambda_factor_2*MatrixXd::Identity(6,6);

		// Lambda_2 << 0.01,  0.0, 0.0, 0.0, 0.0, 0.0,
		// 	    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
	 //          0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
	 //          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
	 //          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
	 //          0.0, 0.0, 0.0, 0.0, 0.0, 0.001;



	auto RLS = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);


    //Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;
	//cutoff frequeny, try frequency of trajectory
	double fc = 20;
	//normalized cutoff frequeency, for butterworth
	double fc_n = fc / 10000;

	//time constant, for lowpassfilter
	double tau = 1/(2*M_PI*fc);
	//damping lowpass filer
	double d = 0.7;

	auto butter_filter_ft = new ButterworthFilter(6);
	butter_filter_ft->setCutoffFrequency(fc_n);
	VectorXd force_moment_butter_filtered = VectorXd::Zero(6);

	auto butter_filter_kin = new ButterworthFilter(6);
	butter_filter_kin->setCutoffFrequency(fc_n);
	VectorXd kin_butter_aux = VectorXd::Zero(6);
	VectorXd kin_butter_filtered = VectorXd::Zero(6);


	//////////////////////////LP///////////////////////////
	VectorXd force_moment_lp_filtered = VectorXd::Zero(6);
	auto low_pass_filter_ft = new am2b::SecOrderLowPass<VectorXd>(VectorXd::Zero(6));
	low_pass_filter_ft->init(1/1000, tau,d);

	auto low_pass_filter_accel = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	low_pass_filter_accel->init(1/1000, tau,d);
	Vector3d accel_lp_filtered = Vector3d::Zero();

	auto low_pass_filter_avel = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	low_pass_filter_avel->init(1/1000, tau,d);

	Vector3d avel_lp_filtered = Vector3d::Zero();
	Vector3d aaccel_lp_filtered = Vector3d::Zero();
	Vector3d avel_butter_filtered = Vector3d::Zero();
	Vector3d accel_butter_filtered = Vector3d::Zero();

	//////////////////////////SVH//////////////////////////////////////

	VectorXd hand_home = VectorXd::Zero(9);
	VectorXd pre_grasp_cylinder = VectorXd::Zero(9);
	VectorXd grasp_cylinder_1 = VectorXd::Zero(9);
	VectorXd grasp_cylinder_2 = VectorXd::Zero(9);

	hand_home << 0.051115,0.094033,0.173342,0.113344,0.171886,0.122578,0.021778,0.006512,0.167929;
	pre_grasp_cylinder << 0.05111,0.989991,0.173342,0.113344,0.171886,0.122578,0.021778,0.006512,0.229999;
	grasp_cylinder_1 << 0.199898,0.989991,0.598884,0.348131,0.505622,0.510044,0.075115,0.268063,0.229999;
	grasp_cylinder_2 << 0.199898,0.989991,0.598884,0.348131,0.505622,0.510044,0.175115,0.268063,0.229999;


	int grasp_counter = 0;
	int grasp_wait_time = 1000;

	//position and orientation for grasping
	Vector3d cylinder_pos_above =  Vector3d(0.678,-0.383, 0.43);
	Vector3d cylinder_pos = Vector3d(0.678,-0.383, 0.267);
	Matrix3d cylinder_ori = Matrix3d::Zero();
	cylinder_ori <<  0.575232,    -0.789174,    -0.215205,
					-0.000949037,  0.262445,    -0.964946,
    				 0.81799,      0.555272,     0.150218;

	//////////////////////////////////MOVEPOINTS//////////////////////////
	Vector3d pos_des_1 = Vector3d(0.5,-0.25, 0.4);
	Vector3d pos_des_2 = Vector3d(0.6,  -0.1, 0.35);



	int state = GOTO_INITIAL_CONFIG;


		//for sinusodial trajectories
	int axis = 4;
	int N = 3;
	double w_s = 1000;
	double w_f = 0.4; 
	VectorXd q_des = VectorXd::Zero(axis);
	VectorXd dq_des = VectorXd::Zero(axis);
	VectorXd ddq_des = VectorXd::Zero(axis);

	VectorXd a = VectorXd::Zero(N*axis);
	VectorXd b = VectorXd::Zero(N*axis);
	// a << -0.0286409,         0.0322926,       0.47785,         -0.571294,       0.0973072,       -0.10507,        -0.194213,       -0.327815,       0.261298,        -0.659976,       0.634429,        0.0897043;
	a<< 0.259086 ,-0.00621783   , 0.429696 ,  -0.028262,   -0.080967  ,  -0.23069 ,  -0.178586 ,   0.267967 ,  -0.023327  ,  0.041493  , -0.304355  , -0.505646;


	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	VectorXd desired_initial_configuration_trunc = VectorXd::Zero(axis);
	desired_initial_configuration_trunc = desired_initial_configuration.tail(axis);
	joint_trajectory->init(desired_initial_configuration_trunc);
	// cout << "desired_initial_configuration_trunc " << desired_initial_configuration_trunc << endl;
	unsigned int long trajectory_counter = 0;
	int trajectory_counter_multiple = 1;
	joint_trajectory->update(trajectory_counter);
	joint_task->_desired_velocity.tail(axis) = joint_trajectory->getJointVelocities();

	//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	ifstream bias;
	bias.open("../../../02-utilities/FT_data1.txt");
	if (!bias)  
	{                     // if it does not work
        cout << "Can't open Data!\n";
    }
    else
    {
    	
    	for (int row ; row<6; row++)
    	{
    		double value = 0.0;
    		bias >> value;
    		force_torque_bias(row) = value;
    	}
    bias.close();
	}

		// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

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
			robot->_ddq =redis_client.getEigenMatrixJSON(JOINT_ACCELERATIONS_KEY);
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
			force_moment -= force_torque_bias;

			robot->rotation(R_link,ee_link); 
			g_local = R_link.transpose()*robot->_world_gravity;

			redis_client.getEigenMatrixDerived(ACCELEROMETER_DATA_KEY, accel);
			redis_client.getEigenMatrixDerived(GYROSCOPE_DATA_KEY, avel);

			accel_local = R_acc_in_ft*accel;
			accel_local *= 9.81;
			accel_local += g_local;
            avel_local = R_acc_in_ft*avel;
            kin_butter_aux << accel_local, avel_local;


			force_moment_butter_filtered =  butter_filter_ft->update(force_moment);
			kin_butter_filtered = butter_filter_kin->update(kin_butter_aux);
			accel_butter_filtered << kin_butter_filtered(0), kin_butter_filtered(1) ,kin_butter_filtered(2);
			avel_butter_filtered << kin_butter_filtered(3), kin_butter_filtered(4), kin_butter_filtered(5);


			accel_lp_filtered = low_pass_filter_accel->process(accel_local);
			avel_lp_filtered = low_pass_filter_avel->process(avel_local);
            aaccel_lp_filtered = low_pass_filter_avel->getDerivative();
            force_moment_lp_filtered = low_pass_filter_ft->process(force_moment);
		}
		// if(state != GOTO_INITIAL_CONFIG)
		// {
			if(controller_counter % 2 == 0)
			{ 
				if(flag_simulation)
				{
					RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);

				}

				else
				{
					RLS->addData(force_moment_butter_filtered, accel_butter_filtered, avel_butter_filtered, Vector3d::Zero(), g_local);
					RLS_2->addData(force_moment_lp_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);					
				}

			phi_RLS = RLS->getInertialParameterVector();
			center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);


			phi_RLS_2 = RLS_2->getInertialParameterVector();
			center_of_mass_RLS_2 << phi_RLS_2(1)/phi_RLS_2(0), phi_RLS_2(2)/phi_RLS_2(0), phi_RLS_2(3)/phi_RLS_2(0); 
			inertia_tensor_RLS_2 << phi_RLS_2(4), phi_RLS_2(5), phi_RLS_2(6), phi_RLS_2(5), phi_RLS_2(7), phi_RLS_2(8), phi_RLS_2(6), phi_RLS_2(8), phi_RLS_2(9);
			}

			if(controller_counter%700==0)
			{
				cout << "current position: " << posori_task->_current_position.transpose() << endl;
				cout << "1 : current inertial parameters for signals butter filtered, zero angular acceleration " << endl; 
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS << endl;
							cout << "ft: " <<  force_moment.transpose() << " a: " << accel_local.transpose() << " omega: "<<  avel_local.transpose() << " alpha: " << aaccel_local.transpose() << " phi " << phi_RLS.transpose() << endl; 

				cout << "2 : current inertial parameters for signals lowpass filtered" << endl; 
		   		cout << "estimated mass: \n" << phi_RLS_2(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS_2.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS_2 << endl;

			}
		// }

		// if(state == ROBOT_FLOAT)
		// {
		// 	// position
		// }
 	 	if(state == GOTO_INITIAL_CONFIG)
		{	
			// update tasks models
			joint_task->_goal_position = desired_initial_configuration;
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			cout << "config error: " << config_error.transpose() << " config error norm: " << config_error.norm() <<endl;  
			if(config_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();

				state =SINUSOIDAL;
				
			}

		}

		if(state == MOVE_TO_CYLINDER)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;


			Vector3d pos_error = cylinder_pos_above - posori_task->_current_position;
			Vector3d ori_error = Vector3d::Zero();
			Sai2Model::orientationError(ori_error, cylinder_ori, posori_task->_current_orientation);
			cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			cout << "ori error: " << ori_error.transpose() << " ori error norm: " << ori_error.norm() <<endl;  
			if(pos_error.norm() < 0.015 && ori_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos;

				state = GRASP_CYLINDER;
			}

		}

		if(state == GRASP_CYLINDER)
		{
			// update tasks models

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques =  joint_task_torques + coriolis + posori_task_torques;

			Vector3d pos_error = cylinder_pos - posori_task->_current_position; 
			// cout << "current position: " << posori_task->_current_position.transpose() <<endl;
			cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			if(pos_error.norm() < 0.01)
			{	
				grasp_counter++;
				if(grasp_counter==grasp_wait_time)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_1);
				}
				if(grasp_counter==grasp_wait_time*2)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_2);
				}

				if(grasp_counter == grasp_wait_time*3)
				{

					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position = cylinder_pos_above + Vector3d(0.0,0.0,0.1);

							
					grasp_counter = 0;		

					cout << "estimation with object" << endl;
					state = MOVE_ABOVE_WITH_OBJECT;
					
				}

			}

		}


		if(state == MOVE_ABOVE_WITH_OBJECT)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;
			Vector3d pos_error = cylinder_pos_above + Vector3d(0.0,0.0,0.1)- posori_task->_current_position; 
			cout << "current position: " << posori_task->_current_position.transpose() <<endl;
			cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  

			if(pos_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos;

				state = LET_GO_OF_CYLINDER;
			}

		}

		if(state == LET_GO_OF_CYLINDER)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques + joint_task_torques + coriolis;
			Vector3d pos_error = cylinder_pos - posori_task->_current_position; 

			cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			if(pos_error.norm() < 0.02)
			{
				grasp_counter++;
				if(grasp_counter==grasp_wait_time)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_1);
				}
				if(grasp_counter==grasp_wait_time*2)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, pre_grasp_cylinder);
				}

				if(grasp_counter==grasp_wait_time*3)
				{
					posori_task->_goal_position(1) += 0.2;
				}


				if(grasp_counter == grasp_wait_time*5)
				{

					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position = pos_des_2;

					grasp_counter = 0;

					state = REST;
					
				}

			}

		}

		if(state == MOVE_CYLINDER_1)
		{
			posori_task->_goal_position = cylinder_pos;
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			Vector3d pos_error = posori_task->_goal_position - posori_task->_current_position; 
			if(pos_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = posori_task->_current_position + Vector3d(-0.1,0.2,0.3);

				state = MOVE_CYLINDER_2;
			}



		}

		if(state == MOVE_CYLINDER_2)
		{

			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			Vector3d pos_error = posori_task->_goal_position - posori_task->_current_position; 
			if(pos_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = posori_task->_current_position + Vector3d(0.1,0.2,0.3);

				state = MOVE_CYLINDER_3;
			}



		}


		if(state == MOVE_CYLINDER_3)
		{

			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			Vector3d pos_error = posori_task->_goal_position - posori_task->_current_position; 
			if(pos_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = posori_task->_current_position + Vector3d(-0.1,-0.2,-0.3);
				state = MOVE_CYLINDER_4;
			}



		}

		if(state == MOVE_CYLINDER_4)
		{

			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			Vector3d pos_error = posori_task->_goal_position - posori_task->_current_position; 
			if(pos_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = posori_task->_current_position + Vector3d(0.1,-0.2,0.3);

				state = LET_GO_OF_CYLINDER;
			}



		}

		if(state == REST)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;
		}


				else if(state == SINUSOIDAL)
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


			joint_task->_goal_position(3) = q_des(0);
			joint_task->_desired_velocity(3)= dq_des(0);

			joint_task->_goal_position(0) = q_des(1);
			joint_task->_desired_velocity(0)= dq_des(1);

			joint_task->_goal_position(5) = q_des(2);
			joint_task->_desired_velocity(5)= dq_des(2);

			joint_task->_goal_position(6) = q_des(3);
			joint_task->_desired_velocity(6)= dq_des(3);

			// mean_accel->process(accel_local_mean, accel_local);
			// mean_avel->process(avel_local_mean, avel_local);
			// mean_aaccel->process(aaccel_local_mean, aaccel_local);
			// mean_g_local->process(g_local_mean, g_local);
			// mean_force_moment->process(force_moment_mean, force_moment);


		
			// RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
			// phi_RLS = RLS->getInertialParameterVector();
			// center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			// inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			// // t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;
			// cout << "Elapsed time trajectory update: " << t_elapsed.count() << endl;

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
					state = REST_SIN;
				}

			}



		}

		else if(state == REST_SIN)
		{

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques   + coriolis;

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
