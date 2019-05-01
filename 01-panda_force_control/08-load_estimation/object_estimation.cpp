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

string DESIRED_POSITION_KEY;
string CURRENT_POSITION_KEY;
string DESIRED_VELOCITY_KEY;
string CURRENT_VELOCITY_KEY;
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

#define SIN_MOTION 12
#define MOVE_ABOVE_OBJECTS 13
#define PRE_LETGO_CYLINDER 14

#define MOVE_LEFT 15
#define BEFORE_OBJECTS 16



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

		DESIRED_POSITION_KEY = "sai2::DemoApplication::Panda::simulation::ee_pos_des";
		CURRENT_POSITION_KEY = "sai2::DemoApplication::Panda::simulation::ee_pos_curr";
		DESIRED_VELOCITY_KEY = "sai2::DemoApplication::Panda::simulation::ee_vel_des";
		CURRENT_VELOCITY_KEY = "sai2::DemoApplication::Panda::simulation::ee_vel_curr";



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
	Vector3d vel_sat = Vector3d(0.18,0.18,0.18);
	Vector3d avel_sat = Vector3d(M_PI/5.5, M_PI/5.5, M_PI/5.5);
	// pos ori controller
	auto posori_task = new Sai2Primitives::PosOriTask(robot, ee_link, ee_pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 95.0;
	posori_task->_kv_pos = 2*sqrt(posori_task->_kp_pos);
	posori_task->_ki_pos = 10;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 2.1*sqrt(posori_task->_kp_ori);
	posori_task->_ki_ori = 10;
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/8.5;
	joint_task->_kp = 60.0;
	joint_task->_kv = 2.1 * sqrt(joint_task->_kp);
	joint_task->_ki = 10;
	VectorXd joint_task_torques = VectorXd::Zero(dof);



	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << -0.35,0,0,-1.5,0,1.5,1.6;
	joint_task->_goal_position = desired_initial_configuration;

	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, ee_link, ee_pos_in_link);
	posori_task2->_max_velocity = 0.1;

	posori_task2->_kp_pos = 55.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_pos);
	posori_task2->_kp_ori = 0.0;
	posori_task2->_kv_ori = 0.0;
	// posori_task->_velocity_saturation = true;
	// posori_task->_linear_saturation_velocity = vel_sat;
	// posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task2_torques = VectorXd::Zero(dof);
	Vector3d initial_position = Vector3d::Zero();
	double sin_motion_counter = 0;
	

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

	int filter_size = 8;
	int filter_size_2 = 10;
//filer

	MatrixXd Lambda_2 = lambda_factor_2*MatrixXd::Identity(6,6);

		Lambda_2 << 0.01,  0.0, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
	          0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
	          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
	          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
	          0.0, 0.0, 0.0, 0.0, 0.0, 0.001;



	auto RLS = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(false,filter_size,Lambda);


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
	// Vector3d cylinder_pos_above =  Vector3d(0.678,-0.383, 0.35);
		Vector3d cylinder_pos_above =  Vector3d(0.678,-0.3, 0.35);

	Vector3d cylinder_pos = Vector3d(0.678,-0.383, 0.267);
	Matrix3d cylinder_ori = Matrix3d::Zero();
	cylinder_ori <<  0.575232,    -0.789174,    -0.215205,
					-0.000949037,  0.262445,    -0.964946,
    				 0.81799,      0.555272,     0.150218;

    Vector3d objects_pos_above = Vector3d(0.6,-0.4, 0.45);
    Vector3d objects_pos_before = Vector3d(0.6,-0.3, 0.3);
   

	//////////////////////////////////MOVEPOINTS//////////////////////////
	Vector3d pos_des_1 = Vector3d(0.5,0.0, 0.3);
	Vector3d pos_des_2 = Vector3d(0.4,-0.3,0.5);
	Vector3d pos_des_3 = Vector3d(0.5,0.1,0.45);

	Matrix3d ori_des_1 = Matrix3d::Zero();
	ori_des_1 << 1 , 0, 0, 
				 0, -1 , 0,
				 0, 0, -1;


	int state = GOTO_INITIAL_CONFIG;

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

	//Gravity compensation
	MatrixXd Jacobian_obj_com = MatrixXd::Zero(3, dof);
	double obj_mass = 1.3;
	Vector3d obj_com = Vector3d(0.0, 0.0, 0.09);

	VectorXd gravity_torques_model = VectorXd(dof);
	VectorXd gravity_torques_clyde = VectorXd(dof);
	VectorXd gravity_torques_object = VectorXd(dof);


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
		if(state != GOTO_INITIAL_CONFIG)
		{
			if(controller_counter % 2 == 0)
			{ 
				if(flag_simulation)
				{
					RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
										RLS_2->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);


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
							// cout << "ft: " <<  force_moment.transpose() << " a: " << accel_local.transpose() << " omega: "<<  avel_local.transpose() << " alpha: " << aaccel_local.transpose() << " phi " << phi_RLS.transpose() << endl; 

				cout << "2 : current inertial parameters for signals lowpass filtered" << endl; 
		   		cout << "estimated mass: \n" << phi_RLS_2(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS_2.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS_2 << endl;

			}
		}

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
			// cout << "config error: " << config_error.transpose() << " config error norm: " << config_error.norm() <<endl;  
			if(config_error.norm() < 0.23)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task2->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos_above;
				posori_task->_desired_orientation = cylinder_ori;
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, pre_grasp_cylinder);
				state = MOVE_TO_CYLINDER;
				
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
			// cout << "MOVE_TO_CYLINDER pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			// cout << "ori error: " << ori_error.transpose() << " ori error norm: " << ori_error.norm() <<endl;  
			if(pos_error.norm() < 0.015 && ori_error.norm() < 0.115)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos;
				posori_task->_desired_orientation = cylinder_ori;

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
			// cout << "GRASP_CYLINDER pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			if(pos_error.norm() < 0.01)
			{	
				grasp_counter++;
				if(grasp_counter==grasp_wait_time)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_1);
				}
				if(grasp_counter==grasp_wait_time*3)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_2);
				}

				if(grasp_counter == grasp_wait_time*5)
				{

					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task2->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position = objects_pos_above;
					// posori_task->_desired_orientation= ori_des_1;

							
					grasp_counter = 0;		

					cout << "estimation with object" << endl;
					// state = MOVE_ABOVE_WITH_OBJECT;
									
				// posori_task2->enableVelocitySaturation(vel_sat,avel_sat);

				
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
			Vector3d pos_error = objects_pos_above - posori_task->_current_position; 
			// cout << "MOVE_ABOVE_WITH_OBJECT current position: " << posori_task->_current_position.transpose() <<endl;
			// cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  

			if(pos_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				// pos_task->enableVelocitySaturation(vel_sat);
				posori_task->_goal_position = pos_des_1;

				// state = LET_GO_OF_CYLINDER;
				// posori_task->_goal_position = pos_des_2;
				// state = SIN_MOTION;
				state = MOVE_CYLINDER_1;

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

			// cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  
			if(pos_error.norm() < 0.01)
			{
				grasp_counter++;
				if(grasp_counter==grasp_wait_time*2)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, grasp_cylinder_1);
				}
				if(grasp_counter==grasp_wait_time*4)
				{
					redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, pre_grasp_cylinder);
				}

				// if(grasp_counter==grasp_wait_time*3)
				// {
				// 	posori_task->_goal_position(1) += 0.2;
				// }


				if(grasp_counter == grasp_wait_time*6)
				{

					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task2->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position(1) += 0.2 ;

					grasp_counter = 0;

					state = BEFORE_OBJECTS;
					
				}

			}

		}

		if(state == MOVE_CYLINDER_1)
		{
			// posori_task->_goal_position = cylinder_pos;
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			Vector3d pos_error = posori_task->_goal_position - posori_task->_current_position; 
			if(pos_error.norm() < 0.15)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = pos_des_2;

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
			if(pos_error.norm() < 0.15)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = pos_des_3;

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
			if(pos_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos_above;
				posori_task->_desired_orientation = cylinder_ori;
				state = PRE_LETGO_CYLINDER;
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


		if(state == SIN_MOTION)
		{
			// update tasks models
			N_prec.setIdentity();
			// posori_task2->updateTaskModel(N_prec);
			// N_prec = pos_task->_N;
			posori_task->updateTaskModel(N_prec);
			
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// position task
			robot->position(initial_position, ee_link, ee_pos_in_link);
			double radius = 0.1;
			double freq = 0.2;
			double sin_motion_time = sin_motion_counter/control_freq;
			// pos_task->_desired_position = initial_position + circle_radius * Eigen::Vector3d(0.0, sin(2*M_PI*circle_freq*sin_motion_time), 1-cos(2*M_PI*circle_freq*sin_motion_time));
			// pos_task->_desired_velocity = 2*M_PI*circle_freq*0.001*Eigen::Vector3d(0.0, cos(2*M_PI*circle_freq*sin_motion_time), sin(2*M_PI*circle_freq*sin_motion_time));
			posori_task->_goal_position =  initial_position + radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*freq*sin_motion_time));
			posori_task->_desired_velocity = 2*M_PI*freq*0.001*Eigen::Vector3d(0.0, 0.0, -cos(2*M_PI*freq*sin_motion_time));
			
			// cout << "desired position: " << pos_task->_goal_position.transpose() << endl;
			// cout << "current position: " << pos_task->_current_position.transpose() << endl;
			// cout << "sin arg: " << freq*sin_motion_time << endl;
			// cout << "sin time: " << sin_motion_time << endl;
			posori_task2->computeTorques(posori_task2_torques);
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task2_torques + posori_task_torques + joint_task_torques + coriolis;
			sin_motion_counter++;
			Vector3d posori_error = posori_task->_goal_position - posori_task->_current_position; 
			if(posori_error.norm() < 0.07)
			{
	
				joint_task->reInitializeTask();
				posori_task2->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = objects_pos_above;
				posori_task->_desired_orientation = cylinder_ori;

				state = MOVE_ABOVE_OBJECTS;
			}
		}

		if(state == MOVE_LEFT)
		{

			// update tasks models
			N_prec.setIdentity();
			// N_prec = pos_task->_N;
			posori_task->updateTaskModel(N_prec);
			
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques =  posori_task_torques + joint_task_torques + coriolis;

			Vector3d posori_error = posori_task->_goal_position - posori_task->_current_position; 
			if(posori_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();
				posori_task2->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = objects_pos_above;
				posori_task->_desired_orientation = cylinder_ori;

				state = MOVE_ABOVE_OBJECTS;
			}



		}

		if(state == MOVE_ABOVE_OBJECTS)
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
			if(pos_error.norm() < 0.015)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos_above;
				posori_task->_desired_orientation = cylinder_ori;

				state = PRE_LETGO_CYLINDER;
			}
		}

		if(state == PRE_LETGO_CYLINDER)
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
			if(pos_error.norm() < 0.01)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position = cylinder_pos;
				posori_task->_desired_orientation = cylinder_ori;

				state = LET_GO_OF_CYLINDER;
			}
		}

		if(state == BEFORE_OBJECTS)
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
			if(pos_error.norm() < 0.015)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				
				

				state =REST;
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

			robot->Jv(Jacobian_obj_com, "link7", obj_com);
			gravity_torques_object = Jacobian_obj_com.transpose() * (- obj_mass * Vector3d(0.0,0.0, -9.81));

			robot->gravityVector(gravity_torques_model);
			gravity_torques_clyde = redis_client.getEigenMatrixJSON(ROBOT_GRAVITY_KEY);

			if(controller_counter % 1000 == 0)
			{
				cout << "gravity_torques_model: " << gravity_torques_model.transpose() << endl;
				cout << "gravity_torques_clyde: " << gravity_torques_clyde.transpose() << endl;
				cout << "gravity_torques_object: " << gravity_torques_object.transpose() << endl;
			}

  

		}



		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		redis_client.setEigenMatrixDerived(CURRENT_POSITION_KEY, posori_task->_current_position);

		redis_client.setEigenMatrixDerived(DESIRED_POSITION_KEY, posori_task->_goal_position);
		redis_client.setEigenMatrixDerived(CURRENT_VELOCITY_KEY, posori_task->_current_velocity);
		redis_client.setEigenMatrixDerived(DESIRED_VELOCITY_KEY, posori_task->_desired_velocity);

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
