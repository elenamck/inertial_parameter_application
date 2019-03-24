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
#define	 PARAMETER_ESTIMATION    1
#define  MOVE_TO_OBJECT          2
#define  MOVE_UP 				 3
#define  MOVE_DOWN 				 4  
#define  MOVE 					 5
#define  HAND_OVER 				 6

/* Data Matrix Force/Torque virtual A_t based on 
 *"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 */
Eigen::MatrixXd GetDataMatrixFT(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local);
/* Data Matrix Force/Torque virtual A_t based on 
 * "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
*/
Eigen::MatrixXd computeSigma(Eigen::MatrixXd K, Eigen::MatrixXd Sigma, Eigen::MatrixXd A);
Eigen::MatrixXd computeK(Eigen::MatrixXd Sigma, Eigen::MatrixXd A, Eigen::MatrixXd Lambda);

void computeInertial(int n_meas, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma); 

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
	Vector3d avel_sat = Vector3d(M_PI/6.0, M_PI/6.0, M_PI/6.0);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.15);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.11;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_ori);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	Matrix3d initial_orientation = Eigen::Matrix3d::Zero();
	Vector3d initial_position = Vector3d::Zero();



	// position controller for angular motion

	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task2->_max_velocity = 0.1;

	posori_task2->_kp_pos = 100.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_pos);
	posori_task2->_kp_ori = 100.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_ori);
	posori_task2->_velocity_saturation = true;
	posori_task2->_linear_saturation_velocity = vel_sat;
	posori_task2->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task2_torques = VectorXd::Zero(dof);
	Vector3d current_position = Vector3d::Zero();

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


	int joint_counter = 0;

	


	Matrix3d R_link = Matrix3d::Zero();

	Vector3d kf_vel = Vector3d::Zero();
	Vector3d accel_kin = Vector3d::Zero();

	//For Inertial Parameter Estimation
	bool linear_case = true;
	bool non_linear_case = false;
	Matrix3d Lambda_lin = 0.01*Matrix3d::Identity();
	MatrixXd Lambda = 0.015* MatrixXd::Identity(6,6);
	// auto LS = new ParameterEstimation::LeastSquare(linear_case);
	auto LS_2 = new ParameterEstimation::LeastSquare(non_linear_case);

	// auto RLS = new ParameterEstimation::RecursiveLeastSquare(linear_case,4,Lambda_lin);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(non_linear_case,7,Lambda);


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

	//debugging purposes
	int n_meas_debug = 0;
	int n_meas_debug_aux = 0;
	int estimation_counter =0;
	MatrixXd A_data_debug = MatrixXd::Zero(6,10);
	VectorXd phi_debug = VectorXd::Zero(10);
	MatrixXd Sigma_debug = Eigen::MatrixXd::Zero(10,10);
	Vector3d center_of_mass_debug = Vector3d::Zero();
	Matrix3d inertia_tensor_debug = Matrix3d::Zero();

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


		LS_2 ->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
		RLS_2 ->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);




		//For Debugging purposes
		// A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
		// computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
		// n_meas_debug++;

		double alpha = 2 * M_PI *joint_counter/control_freq;


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
				robot->rotation(initial_orientation, link_name);
				robot->position(initial_position, link_name, pos_in_link);

				joint_task->reInitializeTask();

			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
				
				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
				center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

				
				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";
			    
			    state = PARAMETER_ESTIMATION;
				
			}
		}



		else if(state == PARAMETER_ESTIMATION)
		{

			initial_orientation << 1,   0,  0,
								   0,  -1,  0,
								   0,   0, -1; 
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			double circle_radius = 0.00004*n_meas_debug_aux;
			double circle_freq = 3;

			double time = estimation_counter/control_freq;

			int orientation_timer = 3000;


			if(estimation_counter < orientation_timer)
			{
				 Eigen::Matrix3d R;
				 double theta = M_PI/2.0;
				R << 1 , 	0, 			 0,
				   	 0 , cos(theta),  sin(theta),
				     0 , -sin(theta), cos(theta) ;

				
				posori_task->_goal_position = initial_position + circle_radius * Eigen::Vector3d(0.0, sin(2*M_PI*circle_freq*time), cos(2*M_PI*circle_freq*time));
				posori_task->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(0.0, cos(2*M_PI*circle_freq*time), -sin(2*M_PI*circle_freq*time));
				
				posori_task->_desired_orientation = R * initial_orientation;

				n_meas_debug_aux++;
			}
			else if(estimation_counter < 1.5*orientation_timer)
			{	

				if (estimation_counter == orientation_timer)
				{
					n_meas_debug_aux=0;
					phi_RLS = RLS_2->getInertialParameterVector();
					center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
					inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
					std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
				    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
				    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
					
					A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
					computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
					n_meas_debug++;
				    center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
					inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
					std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
				    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
				    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

					
					phi_LS = LS_2->getInertialParameterVector();
					center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
					inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
				    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
				    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";

				}
				circle_radius *= 1.3;
				circle_freq *=1.5;
				
				Eigen::Matrix3d R;
				double theta = M_PI/2.0;
				R << cos(theta) , 0 ,- sin(theta),
				 	      0     , 1 ,     0     ,
				     sin(theta) , 0 , cos(theta);
				
				posori_task->_goal_position = initial_position + circle_radius * Eigen::Vector3d( sin(2*M_PI*circle_freq*time), 0.0,cos(2*M_PI*circle_freq*time));
				posori_task->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(cos(2*M_PI*circle_freq*time),0.0,  -sin(2*M_PI*circle_freq*time));


				posori_task->_desired_orientation = R * initial_orientation;

				n_meas_debug_aux++;

			}
			else if(estimation_counter < 2*orientation_timer)
			{	
				if (estimation_counter == 1.5*orientation_timer)
				{
					n_meas_debug_aux=0;
				    phi_RLS = RLS_2->getInertialParameterVector();
					center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
					inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
					std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
				    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
				    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
					
					A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
					computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
					n_meas_debug++;
				    center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
					inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
					std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
				    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
				    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

					
					phi_LS = LS_2->getInertialParameterVector();
					center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
					inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
				    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
				    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
				    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";
				}
				circle_radius *= 1.6;
				circle_freq *=2;
				
				Eigen::Matrix3d R;
				double theta = M_PI/2.0;
				 R <<  cos(theta) ,  sin(theta),  0,
				      -sin(theta) ,  cos(theta),  0,
				            0     ,       0    ,  1;

				posori_task->_goal_position = initial_position + circle_radius * Eigen::Vector3d( sin(2*M_PI*circle_freq*time), cos(2*M_PI*circle_freq*time), 0.0);
				posori_task->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(cos(2*M_PI*circle_freq*time),  -sin(2*M_PI*circle_freq*time), 0.0);
				
				posori_task->_desired_orientation = R * initial_orientation;
				
				n_meas_debug_aux++;

			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			estimation_counter++;
			command_torques = posori_task_torques + joint_task_torques + coriolis;


			if(estimation_counter == 2*orientation_timer)
			{

				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);

				joint_task->reInitializeTask();


			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
				
				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
			    center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

				
				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";

			    current_orientation = posori_task->_current_orientation;

				
				state = MOVE_TO_OBJECT;

			}

		}

		
		else if(state == MOVE_TO_OBJECT)
		{
			// update tasks models

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->_goal_position = Vector3d(.5,0.4,0.3);


			Eigen::Matrix3d R;
				 R << cos(alpha) ,  sin(alpha),  0,
				      -sin(alpha),  cos(alpha),  0,
				           0     ,       0    ,  1;
			posori_task->_desired_orientation = R * current_orientation ;



			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques  + posori_task_torques + coriolis;

			joint_counter++;
			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			// cout << "error_norm double task" << config_error_posori.norm() << endl; 
			if(config_error_posori.norm() < 0.2)
			{	
				joint_counter = 0;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);



			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
				
				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
				center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";

			    current_orientation = posori_task->_current_orientation;


				state = MOVE_UP;		
			}

		}

		else if(state == MOVE_UP)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->_goal_position = Vector3d(.5,0.4,0.9);


			Eigen::Matrix3d R;
				 R <<  cos(alpha) ,  sin(alpha),  0,
				      -sin(alpha) ,  cos(alpha),  0,
				            0     ,       0    ,  1;

			posori_task->_desired_orientation = R * current_orientation;


			// compute torques

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + posori_task_torques + coriolis;

			joint_counter++;



			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			// cout << "error_norm double task" << config_error_posori.norm() << endl; 

			if(config_error_posori.norm() < 0.26)
			{
				joint_counter = 0;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);

			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";

				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
				center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";
				

				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";

			    current_orientation = posori_task->_current_orientation;

				
				state = MOVE_DOWN;
				
			}

		}


		else if(state == MOVE_DOWN)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.5,0.4,0.3);
			Eigen::Matrix3d R;
				 R <<   cos(alpha) ,  sin(alpha),  0,
				      - sin(alpha) ,  cos(alpha),  0,
				             0     ,       0    ,  1;
			
			posori_task->_desired_orientation = R * current_orientation;


			// compute torques

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + posori_task_torques + coriolis;


			joint_counter++;
			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			// cout << "error_norm double task" << config_error_posori.norm() << endl; 

			if(config_error_posori.norm() < 0.26)
			{
				joint_counter = 0;
				posori_task->_desired_orientation = current_orientation;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);

			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
				
				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
				center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";
		
				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";

			    current_orientation = posori_task->_current_orientation;

				
				state = MOVE;
				
			}

		}

		else if(state == MOVE)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			posori_task2->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.5,-0.5,0.3);
			Eigen::Matrix3d R;
				 R <<  cos(alpha) ,  sin(alpha),  0,
				      -sin(alpha) ,  cos(alpha),  0,
				            0     ,       0    ,  1;
			posori_task->_desired_orientation = R * current_orientation;




			robot->position(current_position, posori_task->_link_name, pos_in_link);
			double circle_radius = 0.3;
			double circle_freq = 0.3;
			posori_task2->_goal_position = current_position + circle_radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()));
			posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, 0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()));

			// compute torques

			posori_task->computeTorques(posori_task_torques);
			posori_task2->computeTorques(posori_task2_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + posori_task_torques + posori_task2_torques + coriolis;

			joint_counter++;
			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			// cout << "error_norm double task" << config_error_posori.norm() << endl; 
			if(config_error_posori.norm() < 0.2)
			{
				joint_counter = 0;
				posori_task->_desired_orientation = current_orientation;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);

				joint_task->_goal_position(5) += M_PI/2;

			    phi_RLS = RLS_2->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);
				std::cout << "estimated mass RLS not lin \n" << phi_RLS(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin \n" << 	center_of_mass_RLS.transpose() << "\n";
			    std::cout << "estimated Inertia RLS" << inertia_tensor_RLS << "\n";
				
				A_data_debug = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
				computeInertial(n_meas_debug, force_moment, A_data_debug, phi_debug,  Sigma_debug);
				n_meas_debug++;
				center_of_mass_debug << phi_debug(1)/phi_debug(0), phi_debug(2)/phi_debug(0), phi_debug(3)/phi_debug(0); 
				inertia_tensor_debug << phi_debug(4), phi_debug(5), phi_debug(6), phi_debug(5), phi_debug(7), phi_debug(8), phi_debug(6), phi_debug(8), phi_debug(9);
				std::cout << "estimated mass RLS not lin DEBUG \n" << phi_debug(0) << "\n";
			    std::cout << "estimated center of mass RLS not lin DEBUG \n" << 	center_of_mass_debug.transpose() << "\n";
			    std::cout << "estimated Inertia RLS DEBUG" << inertia_tensor_debug << "\n";

				
				phi_LS = LS_2->getInertialParameterVector();
				center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0);
				inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);
			    std::cout << "estimated mass LS \n" << phi_LS(0) << "\n";
			    std::cout << "estimated center of mass \n" << 	center_of_mass_LS.transpose() << "\n";
			    std::cout << "estimated Inertia LS \n" << inertia_tensor_LS << "\n";
				
				state = HAND_OVER;
				
			}

		}


		else if(state == HAND_OVER)
		{

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			
			N_prec = joint_task->_N_prec; //actually redundant, here joint_task->_N_prec just returns identity matrix!
			posori_task->updateTaskModel(N_prec);

			N_prec = posori_task->_N_prec;
			posori_task2->updateTaskModel(N_prec);

			


			// compute torques
			joint_task->computeTorques(joint_task_torques);
			posori_task->computeTorques(posori_task_torques);
			posori_task2->computeTorques(posori_task2_torques);
			
			// command_torques = joint_task_torques + posori_task_torques + coriolis;

			command_torques = joint_task_torques + posori_task_torques + posori_task2_torques + coriolis;


		}



		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);

		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_LS_KEY, phi_LS);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_DEBUG_KEY, phi_debug);

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

    return 0;

}

Eigen::MatrixXd GetDataMatrixFT(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local)

{
		Eigen::MatrixXd A_t = Eigen::MatrixXd::Zero(6,10);
		for (int i=0; i<3; i++)    
			{
				for (int j=4; j<10; j++)
				{
					A_t(i,j)= 0.0;
				}	
			}
		for (int i=3; i<6; i++)
			{
				A_t(i,0)=0.0;
			} 	  
		A_t(3,1) = 0.0;
		A_t(4,2) = 0.0;
		A_t(5,3) = 0.0;	

		for (int i=0; i<3; i++)
			{
				A_t(i,0) = accel_local(i)-g_local(i);
			}

		A_t(0,1) = - avel_local(1)*avel_local(1) - avel_local(2)*avel_local(2);
		A_t(0,2) = avel_local(0)*avel_local(1) - aaccel_local(2);
		A_t(0,3) = avel_local(0)*avel_local(2) + aaccel_local(1);

		A_t(1,1) = avel_local(0)*avel_local(1) + aaccel_local(2);
		A_t(1,2) = - avel_local(0)*avel_local(0) - avel_local(2)*avel_local(2);  
		A_t(1,3) = avel_local(1)*avel_local(2) - aaccel_local(0);

		A_t(2,1) = avel_local(0)*avel_local(2) - aaccel_local(1);
		A_t(2,2) = avel_local(1)*avel_local(2) + aaccel_local(0);
		A_t(2,3) = - avel_local(1)*avel_local(1)-avel_local(0)*avel_local(0);

		A_t(3,2) = accel_local(2) - g_local(2);  
		A_t(3,3) = g_local(1) - accel_local(1);
		A_t(3,4) = aaccel_local(0);
		A_t(3,5) = aaccel_local(1) - avel_local(0)*avel_local(2);
		A_t(3,6) = aaccel_local(2) + avel_local(0)*avel_local(1);
		A_t(3,7) = - avel_local(1)*avel_local(2);
		A_t(3,8) = avel_local(1)*avel_local(1) - avel_local(2)*avel_local(2);
		A_t(3,9) = avel_local(1)*avel_local(2);

		A_t(4,1) = g_local(2) - accel_local(2);
		A_t(4,3) = accel_local(0) - g_local(0);
		A_t(4,4) = avel_local(0)*avel_local(2);
		A_t(4,5) = aaccel_local(0) + avel_local(1)*avel_local(2);
		A_t(4,6) = avel_local(2)*avel_local(2) - avel_local(0)*avel_local(0);
		A_t(4,7) = aaccel_local(1);
		A_t(4,8) = aaccel_local(2) - avel_local(0)*avel_local(1);
		A_t(4,9) = - avel_local(0)*avel_local(2);

		A_t(5,1) = accel_local(1) - g_local(1);
		A_t(5,2) = g_local(0) - accel_local(0);
		A_t(5,4) = - avel_local(0)*avel_local(1);
		A_t(5,5) = avel_local(0)*avel_local(0) - avel_local(1)*avel_local(1);
		A_t(5,6) = aaccel_local(0) - avel_local(1)*avel_local(2);
		A_t(5,7) = avel_local(0)*avel_local(1);
		A_t(5,8) = aaccel_local(1) + avel_local(0)*avel_local(2);	
		A_t(5,9) = aaccel_local(2);	

		return A_t;

}
Eigen::MatrixXd computeK(Eigen::MatrixXd Sigma, Eigen::MatrixXd A, Eigen::MatrixXd Lambda)
{

	Eigen::MatrixXd K = Sigma*A.transpose()*(A*Sigma*A.transpose()+ Lambda).inverse();

	return K;
}

Eigen::MatrixXd computeSigma(Eigen::MatrixXd K, Eigen::MatrixXd Sigma, Eigen::MatrixXd A)
{

	Sigma = (Eigen::MatrixXd::Identity(10,10) - K*A)*Sigma;
	return Sigma;
}


void computeInertial(int n_meas, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma)
{
	if (n_meas == 0)
	{
		Sigma = Eigen::MatrixXd::Identity(10,10);
	}
	else
	{
		Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(6, 6);
		Eigen::MatrixXd K = computeK(Sigma, A, Lambda);
		Sigma = computeSigma(K, Sigma, A);
		phi = phi + K*(force_virtual - A*phi);
	}
}






