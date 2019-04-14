#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
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
const bool angular_case = true;
const bool inertia_regularization = true;
// redis keys:
// - write:
string JOINT_TORQUES_COMMANDED_KEY;
string LOCAL_GRAVITY_KEY;
string EE_FORCE_SENSOR_FORCE_UNBIASED_KEY;

// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_ACCELERATIONS_KEY;
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
string ANGULAR_ACC_TEST;

string DATA_MATRIX_CURR;
string FT_CURR;

string FORCE_VIRTUAL_DES_KEY;

string JOINT_ANGLE_INPUTS_KEY;
string JOINT_VELOCITIES_INPUTS_KEY;
string JOINT_ACCELERATIONS_INPUTS_KEY;

string ANGULAR_VEL_KEY ;
string ANGULAR_ACC_KEY;
string LINEAR_ACC_KEY;
string LOCAL_GRAVITY_KEY_SIM;



#define  GOTO_INITIAL_CONFIG 	 0
#define  SINUSODIAL				 1
#define  ANGULAR_MOTION			 2
#define	 REST 					 3




int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		JOINT_ACCELERATIONS_KEY = "sai2::DemoApplication::Panda::sensors::ddq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";




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

		//kinematics from simulation
		ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";
		LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";
		LOCAL_GRAVITY_KEY_SIM = "sai2::DemoApplication::Panda::simulation::g_local";

		DATA_MATRIX_CURR = "sai2::DemoApplication::Panda::estimation::current_data_matrix";
		FT_CURR = "sai2::DemoApplication::Panda::estimation::current_data_ft";


		JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
		JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";
		JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::ddq";

		ANGULAR_VEL_TEST = "sai2::DemoApplication::Panda::kinematics::avel::test";
		LINEAR_ACC_TEST = "sai2::DemoApplication::Panda::kinematics::accel::test";
		ANGULAR_ACC_TEST = "sai2::DemoApplication::Panda::kinematics::aaccel::test";


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
	robot->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCELERATIONS_KEY);

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
	const Vector3d pos_in_link = Vector3d(0,0,0.107);


	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);

	joint_task->_max_velocity = M_PI/4;
	joint_task->_kp = 100.0;
	joint_task->_kv = 1.6 * sqrt(joint_task->_kp);


	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	// desired_initial_configuration << 0, 10, 0, -125, 0, 135, 0;


	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;


	Matrix3d current_orientation = Matrix3d::Zero();


	auto ori_task = new Sai2Primitives::OrientationTask(robot, link_name, pos_in_link, Matrix3d::Identity());
	ori_task->_kp = 100.0;
	ori_task->_kv = 20.0;
	ori_task->_ki = 0.0;
	VectorXd ori_task_torques = VectorXd::Zero(dof);




	Matrix3d R_link = Matrix3d::Zero();

	//For Inertial Parameter Estimation
	bool linear_case = true;
	bool non_linear_case = false;
	Matrix3d Lambda_lin = 0.01*Matrix3d::Identity();
	MatrixXd Lambda = 0.014 * MatrixXd::Identity(6,6);

	// auto RLS = new ParameterEstimation::RecursiveLeastSquare(linear_case,4,Lambda_lin);
	auto RLS = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,2,Lambda);
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

	Vector3d accel_test = Vector3d::Zero();
	Vector3d avel_test = Vector3d::Zero();
	Vector3d aaccel_test = Vector3d::Zero();

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

	Vector3d accel_from_sim = Vector3d::Zero();
	Vector3d avel_from_sim = Vector3d::Zero();
	Vector3d aaccel_from_sim = Vector3d::Zero();
	Vector3d g_local_from_sim = Vector3d::Zero();


  	


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
	
	int state = GOTO_INITIAL_CONFIG;

	//for sinusodial trajectories


	// int axis = 4; 
	// int N = 3; 
	int axis = 4;
	int N = 3;
	double w_s = 1000;
	double w_f = 0.8; 
	// double w_f = 0.8; 
	VectorXd q_des = VectorXd::Zero(axis);
	VectorXd dq_des = VectorXd::Zero(axis);
	VectorXd ddq_des = VectorXd::Zero(axis);

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
	
// a <<-0.590865, 	0.274173, 	-0.242807, 	-0.530158, 	-0.549219, 	0.537474, 	0.0907198, 	-0.234161, 	-0.374772, 	0.0721155, 	0.191923, 	-0.00187571;	
// a << 	-0.418301, 	-0.121738, 	0.964821;	
a << -0.0286409, 	0.0322926, 	0.47785, 	-0.571294, 	0.0973072, 	-0.10507, 	-0.194213, 	-0.327815, 	0.261298, 	-0.659976, 	0.634429, 	0.0897043;


// b <<0.164893, 	0.286424, 	-0.270565, 	0.686873, 	-0.102795, 	0.343893, 	0.0232298, 	-0.136117, 	-0.589154, 	0.182915, 	0.389056, 	0.473262;

	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	VectorXd desired_initial_configuration_trunc = VectorXd::Zero(axis);
	desired_initial_configuration_trunc = desired_initial_configuration.tail(axis);
	joint_trajectory->init(desired_initial_configuration_trunc);
	cout << "desired_initial_configuration_trunc " << desired_initial_configuration_trunc << endl;
	unsigned int long trajectory_counter = 0;
	int trajectory_counter_multiple = 1;
	joint_trajectory->update(trajectory_counter);
	joint_task->_desired_velocity.tail(axis) = joint_trajectory->getJointVelocities();



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
		robot->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCELERATIONS_KEY);
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);


		//Kinematics 





		
		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);


			accel_from_sim = redis_client.getEigenMatrixJSON(LINEAR_ACC_KEY);
			avel_from_sim = redis_client.getEigenMatrixJSON(ANGULAR_VEL_KEY);
			aaccel_from_sim = redis_client.getEigenMatrixJSON(ANGULAR_ACC_KEY);
			g_local_from_sim = redis_client.getEigenMatrixJSON(LOCAL_GRAVITY_KEY_SIM);
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

		// mean_accel->process(accel_local_mean, accel_local);
		// mean_avel->process(avel_local_mean, avel_local);
		// mean_aaccel->process(aaccel_local_mean, aaccel_local);
		// mean_g_local->process(g_local_mean, g_local);
		// mean_force_moment->process(force_moment_mean, force_moment);


			// RLS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
			// A_curr = RLS->getCurrentDataMatrix();
			// ft_curr = RLS->getCurrentInputVector(); 


		




		if(state == GOTO_INITIAL_CONFIG)
		{	

			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.01)
			{

				joint_trajectory->init(desired_initial_configuration_trunc);

				
					cout << "Initial Config reached" << endl;
			    	state = SINUSODIAL;
				

			    // joint_task->_goal_position(6) = 0.8*M_PI;
				
				
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

			



			// LS->addData(force_moment_mean, accel_local_mean, avel_local_mean, aaccel_local_mean, g_local_mean);
			// LS->addData(force_moment, accel_test, avel_test, aaccel_test, g_local);
// RLS->addData(force_moment, accel_kin, avel_kin, aaccel_kin, g_local);
			// t_start = std::chrono::high_resolution_clock::now();

			RLS->addData(force_moment, accel_from_sim, avel_from_sim, aaccel_from_sim, g_local_from_sim);
			phi_RLS = RLS->getInertialParameterVector();
			center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			// t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;
			// cout << "Elapsed time trajectory update: " << t_elapsed.count() << endl;

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

			if ((trajectory_counter/w_s) >= trajectory_counter_multiple *(2*M_PI/w_f)/2 )
			{
				cout << "excictation period finished" << endl;
				trajectory_counter_multiple ++; 
				
				// phi_LS = LS->getInertialParameterVector();
				// center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
				// inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				// cout << "estimated mass LS: \n" << phi_LS(0) << "\n";
		  //  		cout << "estimated center of mass LS: \n" << 	center_of_mass_LS.transpose() << "\n";
		  //   	cout << "estimated Inertia LS: \n" << inertia_tensor_LS << "\n";

		    	// cout << "datamatrix: \n" << LS->getCurrentDataMatrixStacked() << endl;
		    	// cout << "FT Vector: \n" << LS->getCurrentInputVectorStacked() << endl;
		    	



		    	// state = REST;
		    	// joint_task->_goal_position = desired_initial_configuration;
			}



		}

		else if(state == ANGULAR_MOTION)
		{
			N_prec.setIdentity();
			ori_task->updateTaskModel(N_prec);
			N_prec = ori_task->_N;
			joint_task->updateTaskModel(N_prec);
			Matrix3d desired_orientation = Matrix3d::Zero();
			desired_orientation << 
			cos(controller_counter/control_freq), -sin (controller_counter/control_freq) , 0,
			sin(controller_counter/control_freq), cos(controller_counter/control_freq), 0, 
			0									, 	0								, 1;


			desired_orientation = R_link * desired_orientation;
			// ori_task->_desired_orientation = desired_orientation;
			joint_trajectory->update(controller_counter);
			VectorXd joint_vel_des = joint_trajectory->getJointVelocities();
			double joint_vel_des_entry = joint_vel_des(0);
			ori_task->_desired_angular_velocity = R_link * Vector3d(0.0, 0.0, joint_vel_des_entry);


			// if(controller_counter % 1000 == 0)
			// {
			// 	cout << "joint acceleration" << robot->_ddq.transpose() << endl;
			// 	cout << "computed acceleration" << aaccel_kin.transpose() << endl; 
			// 	cout << "desired joint velocity" << joint_vel_des_entry << endl; 


			// }
			avel_test = avel_kin;
			aaccel_test = aaccel_kin;
			avel_test(0) = 0;
			avel_test(1) = 0;
			aaccel_test(0) = 0;
			aaccel_test(1) = 0; 

			RLS->addData(force_moment, accel_test, avel_test, aaccel_test, g_local);
			phi_RLS = RLS->getInertialParameterVector();
			center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
			

			if(controller_counter%1000==0)
		{
			cout << "estimated mass: \n" << phi_RLS(0) << "\n";
		    cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << "\n";
		    cout << "estimated Inertia: \n" << inertia_tensor_RLS << "\n";

		}

			ori_task->computeTorques(ori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = ori_task_torques + joint_task_torques + coriolis;


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


		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_UNBIASED_KEY, force_moment);

		redis_client.setEigenMatrixDerived(ANGULAR_VEL_TEST, avel_test);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_TEST, accel_test);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_TEST, aaccel_test);

		redis_client.setEigenMatrixDerived(JOINT_ANGLE_INPUTS_KEY, q_des);

		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_INPUTS_KEY, dq_des);
		redis_client.setEigenMatrixDerived(JOINT_ACCELERATIONS_INPUTS_KEY, ddq_des);

		
  //  		 double end_time = timer.elapsedTime();
  //   	cout << "\n";
  //   cout << "Loop run time  : " << end_time << " seconds\n";
  //   cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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



