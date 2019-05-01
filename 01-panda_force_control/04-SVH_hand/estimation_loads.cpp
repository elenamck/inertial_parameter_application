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

string goal_position_KEY;
string goal_position_2_KEY;

string DESIRED_VELOCITY_2_KEY;
string DESIRED_VELOCITY_KEY;

string CURRENT_POSITION_KEY;
string CURRENT_VELOCITY_KEY;

string CURRENT_ORIENTATION_KEY;
string DESIRED_ORIENTATION_KEY;
string CONTROLLER_GAINS_KEY;
#define  GOTO_INITIAL_CONFIG 	 	0
#define	 AUX 						1
#define  PRE_PICK					2
#define  PICK						3
#define	 CARRY 						4
#define	 PRE_PLACE					5
#define  PLACE 						6
#define	 REST						7


Matrix3d getRotMatrix(Matrix3d rot_not_unit);

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

		goal_position_KEY = "sai2::DemoApplication::Panda::simulation::ee_pos_des";
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

		goal_position_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_pos_des";
		// goal_position_2_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_pos_des";

		CURRENT_POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_pos_curr";
		// DESIRED_VELOCITY_2_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_vel_des";

		DESIRED_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_vel_des";
		CURRENT_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_vel_curr";

		CURRENT_ORIENTATION_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_ori_curr";
		DESIRED_ORIENTATION_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_ori_des";
		CONTROLLER_GAINS_KEY = "sai2::DemoApplication::FrankaPanda::controller::gains";

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
	Vector3d vel_sat = Vector3d(0.1,0.1,0.1);
	Vector3d avel_sat = Vector3d(M_PI/7, M_PI/7, M_PI/7);
	// pos ori controller
	auto pos_task = new Sai2Primitives::PositionTask(robot, ee_link, ee_pos_in_link);
	pos_task->_max_velocity = 0.12;
	pos_task->_kp = 120.0;
	pos_task->_kv = 7;
	pos_task->_ki = 5;
	pos_task->enableVelocitySaturation(vel_sat);

	// pos_task->_velocity_saturation = false;
	VectorXd pos_task_torques = VectorXd::Zero(dof);

	auto ori_task = new Sai2Primitives::OrientationTask(robot, ee_link, ee_pos_in_link);
	ori_task->_kp = 135.0;
	ori_task->_kv = 25;
	ori_task->_ki = 15;
	VectorXd ori_task_torques = VectorXd::Zero(dof);
	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/8.5;
	joint_task->_kp = 62.0;
	joint_task->_kv = 2.0 * sqrt(joint_task->_kp);
	joint_task->_ki = 5;
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd controller_gains = VectorXd::Zero(9);
	controller_gains << joint_task->_kp, joint_task->_kv, joint_task->_ki, pos_task->_kp, pos_task->_kv,pos_task->_ki,pos_task->_kp, pos_task->_kv,pos_task->_ki;
	redis_client.setEigenMatrixDerived(CONTROLLER_GAINS_KEY, controller_gains);

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << -0.35,0,0,-1.5,0,1.5,1.6;
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

	double lambda_factor = 0.1;
	double lambda_factor_2 = 0.002;
	MatrixXd Lambda = lambda_factor*MatrixXd::Identity(6,6);

	int filter_size = 4;
	int filter_size_2 = 12;
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
	double d = 0.8;

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
	pre_grasp_cylinder  << 0.05111,0.989991,0.173342,0.113344,0.171886,0.122578,0.021778,0.006512,0.22999;
	grasp_cylinder_1 << 0.199898,0.989991,0.598884,0.348131,0.505622,0.510044,0.075115,0.268063,0.22999;
	grasp_cylinder_2 << 0.199898,0.989991,0.598884,0.348131,0.505622,0.510044,0.175115,0.268063,0.229999;

	int grasp_counter = 0;
	int grasp_wait_time = 1000;


	Quaterniond rot_quat_curr; 
	Quaterniond rot_quat_des;

	VectorXd rot_quat_curr_redis = VectorXd::Zero(4); 
	VectorXd rot_quat_des_redis = VectorXd::Zero(4);

	Matrix3d helper = Matrix3d::Zero();
	Vector3d aux_pos = Vector3d(0.7,0.0,0.45);
	Matrix3d aux_ori = Matrix3d::Zero();
	helper << 0.518496, -0.617,-0.01,
              -0.43, 0.121,-0.9895,
               0.74, 0.778,0.144;
    aux_ori = getRotMatrix(helper);

    Vector3d pick_pos_before = Vector3d(0.7,-0.4,0.25);
    Matrix3d pick_ori_before = Matrix3d::Zero();
    helper << 0.787, -0.617, -0.01,
    		0.078, 0.121,-0.9895;
            0.6121, 0.778,0.144;
    pick_ori_before = getRotMatrix(helper);

    Vector3d pick_pos = Vector3d(0.7 ,-0.4 , 0.25);
    // Matrix3d pick_ori = Matrix3d::Zero();
    // helper << 0.37 ,  -0.84 ,  -0.,
    //            -0.008 ,  0.43,   -0.9,
    //              0.93,   0.33  , 0.15;
    // pick_ori = getRotMatrix(helper);
    

  	Matrix3d pick_ori_second = Matrix3d::Zero();
  	helper <<  0.54  , -0.75, -0.4,
                    -0.022 , 0.45 , -0.9 ,
                     0.84,  0.49 ,  0.225;
	pick_ori_second = getRotMatrix(helper);

    Vector3d pick_pos_above = Vector3d(0.7,-0.39 , 0.265);
    Matrix3d pick_ori_above = Matrix3d::Zero();
    helper << 0.66,  -0.67 ,  -0.335,
             -0.23  , -0.6 , -0.758,
              0.71 ,  0.42,  -0.558;
 	pick_ori_above = getRotMatrix(helper);
 	
    Vector3d carry_pos_1 = Vector3d(0.64 , -0.11, 0.403);
    Matrix3d carry_ori_1 = Matrix3d::Zero();
    helper << 0.383 , -0.888 , 0.251,
                  -0.62  , -0.449 , -0.643,
                  -0.685 ,  0.0906996 , -0.723;
    carry_ori_1 = getRotMatrix(helper);

    Vector3d carry_pos_2 = Vector3d(0.6,0.0,0.43);
    Matrix3d carry_ori_2 = Matrix3d::Zero();
    helper << 0.9 , -0.4044, 0.131,
                  -0.42  , -0.879 , 0.218,
                   0.03,  -0.253 , -0.967;
    carry_ori_2 = getRotMatrix(helper);

    Vector3d carry_pos_3 = Vector3d(0.54,0.2,0.4);
    Matrix3d carry_ori_3 = Matrix3d::Zero();
    helper<< 0.9 , -0.345, -0.28,
             	 -0.054  , -0.713 , 0.7,
             	  0.44,  -0.61 , -0.66;
    carry_ori_3 = getRotMatrix(helper);

    Vector3d carry_pos_4 = Vector3d(0.51,0.24,0.407);
    Matrix3d carry_ori_4 = Matrix3d::Zero();
    helper << 0.465, -0.665, -0.584,
                  0.252, -0.533 , 0.8,
                  0.848,  -0.523 , -0.08;
	carry_ori_4 = getRotMatrix(helper);




    Vector3d place_pos_above= Vector3d(0.48, 0.44,0.303);
    Matrix3d place_ori_above= Matrix3d::Zero();
    helper << 0.465, -0.665, -0.584,
                     0.252, -0.533 , 0.8,
                     0.848,  -0.523 , -0.08;
    place_ori_above = getRotMatrix(helper);

    Vector3d place_pos= Vector3d(0.48, 0.44,0.303);
    Matrix3d place_ori= Matrix3d::Zero();
    helper << 0.38, -0.39,  -0.837,
         			   0.519,-0.6588, 0.545,
        			  -0.766, 0.64, -0.0;
    place_ori = getRotMatrix(helper);

    int carry_counter = 0;

    Vector3d pick_pos_square_long = Vector3d(0.58, -0.33, 0.29);
  	Matrix3d pick_ori_square_long = Matrix3d::Zero();

	 pick_ori_square_long << 0.24308995,  -0.9296554458 , -0.2768531426,
	 						-0.1036049802,   0.2588968457,  -0.9603324945,
	 						 0.9644548156,   0.2621308437, -0.03338191719;

    Vector3d pick_pos_square = Vector3d(0.374, -0.402, 0.426);
  	Matrix3d pick_ori_square = Matrix3d::Zero();
  	pick_ori_square <<   0.0148661,   -0.9718,  0.235336,
 						 -0.911008, -0.110172,   -0.3974,
   						0.412121, -0.208485, -0.886956;

    Matrix3d pick_ori = Matrix3d::Zero();
    pick_ori << 0.07 ,  -0.84 ,  -0.,
            	  0 ,  0.2,   -0.9,
              0.93,   0.2  ,0.316227766;

	/////////////////////////////////////////////////////////
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
				cout << "current position: " << pos_task->_current_position.transpose() << endl;
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
			if(config_error.norm() < 0.22)
			{
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();

				pos_task->_goal_position = aux_pos;
				pos_task->enableVelocitySaturation(vel_sat);
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, pre_grasp_cylinder);
				state = AUX;
				
			}

		}
		if(state == AUX)
		{
			// update tasks models
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
	
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = pos_task_torques+ joint_task_torques + coriolis ;


			Vector3d pos_error = pos_task->_goal_position - pos_task->_current_position;

			if(pos_error.norm() < 0.001 )
			{
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				pos_task->_goal_position = pick_pos_before;
				state = PRE_PICK;

			}

		}
		if(state == PRE_PICK)
		{
			// update tasks models
			N_prec.setIdentity();

			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = pos_task_torques+ joint_task_torques + coriolis;


			Vector3d pos_error = pos_task->_goal_position - pos_task->_current_position;
			if(pos_error.norm() < 0.01 )
			{
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				pos_task->_goal_position = pick_pos;
				ori_task->_desired_orientation = pick_ori;
				// pos_task->_desired_orientation = pick_ori;

				state = PICK;
			}
		}
		if(state == PICK)
		{
			// update tasks models
			// update tasks models
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);			
			ori_task->updateTaskModel(N_prec);

			N_prec = pos_task->_N;

			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);
			ori_task->computeTorques(ori_task_torques);

			command_torques = pos_task_torques+ joint_task_torques + ori_task_torques+ coriolis;


			Vector3d pos_error = pos_task->_goal_position - pos_task->_current_position;
			if(pos_error.norm() < 0.01 )
			{	
				grasp_counter++;
				if(grasp_counter==grasp_wait_time*2)
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
					pos_task->reInitializeTask();
					pos_task->_goal_position = carry_pos_1;
					// pos_task->_desired_orientation =carry_ori_1;
					state = CARRY;
				}

		}



		}
		if(state == CARRY)
		{
			// update tasks models

			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques =  joint_task_torques + coriolis + pos_task_torques;

			Vector3d pos_error = pos_task->_goal_position - pos_task->_current_position; 
			if(pos_error.norm() < 0.01)
			{	
				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				carry_counter ++;
				if(carry_counter == 1)
				{
					pos_task->_goal_position = carry_pos_2;
					// pos_task->_desired_orientation =carry_ori_2;cc
				}
				else if(carry_counter == 2)
				{
					pos_task->_goal_position = carry_pos_3;
					// pos_task->_desired_orientation =carry_ori_3;
				}
				else if(carry_counter == 3)
				{
					pos_task->_goal_position = carry_pos_4;
					// pos_task->_desired_orientation =carry_ori_4;
				}
				else if(carry_counter > 3)
				{
					pos_task->_goal_position = place_pos_above;
					// pos_task->_desired_orientation =place_ori_above;
					carry_counter = 0;
					state = PRE_PLACE;
				}

				
			}
			
		}
		
		if(state == PRE_PLACE)
		{	
			// update tasks models
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = pos_task_torques+ joint_task_torques + coriolis;
			Vector3d pos_error = pos_task->_goal_position- pos_task->_current_position; 
			cout << "pos error: " << pos_error.transpose() << " pos error norm: " << pos_error.norm() <<endl;  

			if(pos_error.norm() < 0.01)
			{	

					joint_task->reInitializeTask();
					pos_task->reInitializeTask();
					pos_task->_goal_position = place_pos;
					// pos_task->_desired_orientation = place_ori;  

					state = PLACE;
			}

		}

		if(state == PLACE)
		{
			// update tasks models
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = pos_task_torques + joint_task_torques + coriolis;
			Vector3d pos_error = pos_task->_goal_position - pos_task->_current_position; 
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

				if(grasp_counter==grasp_wait_time*5)
				{
					pos_task->_goal_position(0) += 0.1;
				}

				if(grasp_counter==grasp_wait_time*5)
				{
					pos_task->_goal_position(1) += 0.1;
				}


				if(grasp_counter == grasp_wait_time*7)
				{

					joint_task->reInitializeTask();
					pos_task->reInitializeTask();

					state = REST;
					
				}

			}

		}

		if(state == REST)
		{
			// update tasks models
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques =pos_task_torques+ joint_task_torques + coriolis;

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

		// rot_quat_des = pos_task->_desired_orientation;
		// rot_quat_curr = pos_task->_current_orientation;
		rot_quat_des_redis << rot_quat_des.w(), rot_quat_des.vec() ;
		rot_quat_curr_redis << rot_quat_curr.w(), rot_quat_curr.vec() ;


		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		redis_client.setEigenMatrixDerived(CURRENT_POSITION_KEY, pos_task->_current_position);

		redis_client.setEigenMatrixDerived(goal_position_KEY, pos_task->_goal_position);
		// redis_client.setEigenMatrixDerived(goal_position_2_KEY, pos_task2->_goal_position);

		redis_client.setEigenMatrixDerived(CURRENT_VELOCITY_KEY, pos_task->_current_velocity);
		
		redis_client.setEigenMatrixDerived(DESIRED_VELOCITY_KEY, pos_task->_desired_velocity);
		// redis_client.setEigenMatrixDerived(DESIRED_VELOCITY_2_KEY, pos_task2->_desired_velocity);

		redis_client.setEigenMatrixDerived(CURRENT_ORIENTATION_KEY, rot_quat_curr_redis);
		
		redis_client.setEigenMatrixDerived(DESIRED_ORIENTATION_KEY, rot_quat_des_redis);


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
Matrix3d getRotMatrix(Matrix3d rot_not_unit)
{
	Matrix3d rot_unit = Matrix3d::Zero();

	for( int i = 0;i<3;i++)
	{
		rot_unit.row(i) = rot_not_unit.row(i).normalized();
	}
	return rot_unit;
}
