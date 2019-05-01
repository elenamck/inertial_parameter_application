#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "filters/ButterworthFilter.h"
#include "filters/SecOrderLowPass.hpp"




#include <signal.h>
#include <iostream>
#include <stdlib.h> 
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
std::string SVH_HAND_POSITION_COMMAND_KEY;
std::string SVH_HAND_GRASP_FLAG_KEY;

// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_ACCELERATIONS_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;
std::string ACCELEROMETER_DATA_KEY;
std::string GYROSCOPE_DATA_KEY;
std::string SVH_HAND_POSITIONS_KEY;


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

string 	LINEAR_ACC_LP_KEY;
string	ANGULAR_VEL_LP_KEY;
string	ANGULAR_ACC_LP_KEY;
string EE_FORCE_SENSOR_LP_KEY;

string 	LINEAR_ACC_BUTTER_KEY;
string	ANGULAR_VEL_BUTTER_KEY;
string EE_FORCE_SENSOR_BUTTER_KEY;

string INERTIAL_PARAMS_LP_KEY; 
string INERTIAL_PARAMS_BUTTER_KEY;

#define  GOTO_INITIAL_CONFIG 	 	0
#define  MOVE_TO_OBJECT          	1
#define  MOVE_ABOVE_OBJECT 		 	2
#define  MOVE_DOWN_TO_OBJECT 	 	3  
#define  GRASP 					 	4
#define  MOVE_ABOVE_WITH_OBJECT  	5
#define  MOVE_DOWN_WITH_OBJECT 	 	6
#define  LET_GO_OF_OBJECT 		 	7
#define	 MOVE_BACK					8
#define  GO_BACK_TO_INITIAL_CONFIG 	9


#define CALIB_UP 10
#define CALIB_DOWN 11
#define MOVE_SIN 12



int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		JOINT_ACCELERATIONS_KEY = "sai2::DemoApplication::Panda::sensors::ddq";

		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";

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

		//corrected sensor data(accelerometer: gravity removed, right frame, Gyroscope: right frame)
		// LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
		// ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
		EE_FORCE_SENSOR_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";
		// QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
		// POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";

		SVH_HAND_POSITION_COMMAND_KEY ="sai2::SVHHand_Left::position_command";
		SVH_HAND_GRASP_FLAG_KEY ="sai2::SVHHand_Left::grasp_type_flag";
		SVH_HAND_POSITIONS_KEY = "sai2::SVHHand_Left::position"; 


		LINEAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel::lowpass";
		ANGULAR_VEL_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel::lowpass";
		ANGULAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel::lowpass";


		LINEAR_ACC_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel::butter";
		ANGULAR_VEL_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel::butter";

		EE_FORCE_SENSOR_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::force_moment::lowpass";

		EE_FORCE_SENSOR_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::force_moment::butter";

		INERTIAL_PARAMS_LP_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::lowpass";
		INERTIAL_PARAMS_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::butter";
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::phi";

		LOCAL_GRAVITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::g_local";

		
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
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq );
	if(flag_simulation)
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
	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	Vector3d avel_sat = Vector3d(M_PI/4, M_PI/4, M_PI/4);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 90.0;
	posori_task->_kv_pos = 2*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 40.0;
	posori_task->_kv_ori = 2*sqrt(posori_task->_kp_ori);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// position controller for angular motion


	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/8.5;
	// joint_task->_max_velocity = 0;
	joint_task->_kp = 75.0;
	joint_task->_kv = 2.1*sqrt(joint_task->_kp);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	VectorXd desired_grasping_configuration = VectorXd::Zero(dof);
	VectorXd desired_letgo_configuration = VectorXd::Zero(dof);

	VectorXd calib_down= VectorXd::Zero(dof);
	VectorXd calib_up= VectorXd::Zero(dof);

	calib_down << -0.212803,-0.894344,-0.323602,-2.45985,-0.240155,1.59861,2.3368;
	calib_up = calib_down;
	calib_up(4) -= M_PI;


	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	pos_task->_max_velocity = 0.1;
	pos_task->_kp = 60.0;
	pos_task->_kv = 2*sqrt(posori_task->_kp_pos);
	VectorXd pos_task_torques = VectorXd::Zero(dof);
	Vector3d current_position = Vector3d::Zero();


	
	
	

	
	desired_initial_configuration << 0.0614699,-0.275042,-0.376448,-1.74132,-0.0623806,1.51744,-2.02767;
	desired_grasping_configuration << 0.333325,0.484366,-0.310534,-1.62655,-1.3009,1.24677,-1.80465;
	desired_letgo_configuration << 0.303131,0.863745,-0.910716,-1.63203,-1.04949,2.0947,-1.64691;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	//For Inertial Parameter Estimation
	int n_measurements = 0;
	Vector3d accel = Eigen::Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Eigen::Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Eigen::Vector3d::Zero(); //object angular acceleration in base frame
	Vector3d accel_local = Eigen::Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Eigen::Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Eigen::Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Eigen::Vector3d::Zero(); //gravity vector in sensor frame
	MatrixXd A_data = Eigen::MatrixXd::Zero(6,10); //Data matrix
	VectorXd phi = Eigen::VectorXd::Zero(10); //inertial parameter vector
	MatrixXd Sigma = Eigen::MatrixXd::Zero(10,10);
	Matrix3d inertia_tensor = Eigen::Matrix3d::Zero();
	Vector3d center_of_mass = Eigen::Vector3d::Zero();

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

    Q.diagonal() << 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4;
    R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-3, 1.0e-3, 1.0e-3;
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

	VectorXd hand_home = VectorXd::Zero(9);
	VectorXd hand_pre_grasp = VectorXd::Zero(9);
	VectorXd hand_grasp_1 = VectorXd::Zero(9);
	VectorXd hand_grasp_2 = VectorXd::Zero(9);
	VectorXd hand_grasp_3 = VectorXd::Zero(9);
	VectorXd hand_grasp_4 = VectorXd::Zero(9);
	VectorXd hand_grasp_5 = VectorXd::Zero(9);
	VectorXd hand_grasp_6 = VectorXd::Zero(9);
	
	VectorXd current_position_hand = VectorXd::Zero(9);

	VectorXd hand_let_go_1 = VectorXd::Zero(9);
	VectorXd hand_let_go_2 = VectorXd::Zero(9);


	double thumb_flex_home  	 = 0.052184;
	double thumb_oppo_home  	 = 0.094618;
	double index_dist_home  	 = 0.173368;
	double index_prox_home  	 = 0.113306;
	double middle_dist_home  	 = 0.171912;
	double middle_prox_home 	 = 0.121707;
	double ring_home  		 	 = 0.021778;
	double pinky_home		     = 0.006512;
	double finger_spread_home 	 = 0.167904;

	double thumb_flex_grasp  	 = 0.28;
	double thumb_oppo_grasp 	 = 0.99;
	double index_dist_grasp 	 = 0.9;
	double index_prox_grasp 	 = 0.7;
	double middle_dist_grasp  	 = 0.55;
	double middle_prox_grasp	 = 0.8;
	double ring_grasp 		 	 = 0.34;
	double pinky_grasp		     = 0.4;
	double finger_spread_grasp  = 0.35;

	double thumb_flex_aux		 = 0.15;
	double index_prox_aux        = 0.3;
	double index_dist_aux		 = 0.5;
	double middle_prox_aux		 = 0.3;

	hand_home << thumb_flex_home, thumb_oppo_home, index_dist_home, index_prox_home, middle_dist_home, middle_prox_home, ring_home, pinky_home, finger_spread_home;  	 
	hand_pre_grasp << thumb_flex_home, thumb_oppo_grasp, index_dist_home, index_prox_home, middle_dist_home, middle_prox_home, ring_home, pinky_home, finger_spread_grasp;
	hand_grasp_1 <<	thumb_flex_aux, thumb_oppo_grasp, index_dist_home, index_prox_aux, middle_dist_home, middle_prox_home, ring_home, pinky_home, finger_spread_grasp;
	hand_grasp_2 << thumb_flex_aux, thumb_oppo_grasp, index_dist_aux, index_prox_grasp, middle_dist_home, middle_prox_home, ring_home, pinky_home, finger_spread_grasp;
	hand_grasp_3 << thumb_flex_grasp, thumb_oppo_grasp, index_dist_grasp, index_prox_grasp, middle_dist_home, middle_prox_aux, ring_home, pinky_home, finger_spread_grasp;
	hand_grasp_4 << thumb_flex_grasp, thumb_oppo_grasp, index_dist_grasp, index_prox_grasp, middle_dist_grasp, middle_prox_grasp, ring_home, pinky_home, finger_spread_grasp;
	hand_grasp_5 << thumb_flex_grasp, thumb_oppo_grasp, index_dist_grasp, index_prox_grasp, middle_dist_grasp, middle_prox_grasp, ring_home, pinky_grasp, finger_spread_grasp;
	hand_grasp_6 << thumb_flex_grasp, thumb_oppo_grasp, index_dist_grasp, index_prox_grasp, middle_dist_grasp, middle_prox_grasp, ring_grasp, pinky_grasp, finger_spread_grasp;

	hand_let_go_1 << thumb_flex_grasp, thumb_oppo_grasp, index_dist_grasp, index_prox_grasp, middle_dist_grasp, middle_prox_grasp, ring_home, pinky_home, finger_spread_grasp;
	hand_let_go_2 << thumb_flex_home, thumb_oppo_grasp, 0.0, index_prox_home, 0.0, middle_prox_home, ring_home, pinky_home, finger_spread_grasp;
	//cout << "hand_pre_grasp" << hand_pre_grasp.transpose() << endl;
	int n_counter = 0;
	int wait_time = 800;
	Vector3d desired_position_above_hand = Vector3d(0.655, -0.2,  0.4);
	Vector3d desired_position_letgo_hand = Vector3d(0.67, -0.22, 0.30);
	Matrix3d desired_orientation_above_hand = Matrix3d::Zero();
	Vector3d desired_position_move_with_obj_up = Vector3d(0.2, -0.2 , 0.65);
	Vector3d desired_position_move_with_obj_down = Vector3d(0.4, -0.1 , 0.6);

	double theta = M_PI/1.5;
	Matrix3d desired_orientation_with_obj_up = Matrix3d::Identity();

Matrix3d desired_orientation_z_down = Matrix3d::Zero();
 desired_orientation_z_down << 1, 0, 0, 
							  0, -0.707, 0.707, 
							0, -0.707 ,-0.707;	// desired_orientation_with_obj_up << 	 1  ,0, 0,
 // 										0 , cos(theta) , -sin(theta), 
 // 										0 , sin(theta) ,cos(theta);



 	
Matrix3d desired_orientation_at_obj = Matrix3d::Zero();
	desired_orientation_at_obj << -.7,0.3, 1,0,
 								 0  ,0,     -1, 
 								-0.3,	0.7,0;

 	Vector3d ori_error = Vector3d::Zero();
 	Vector3d pos_error = Vector3d::Zero();
 	Vector3d posori_error = Vector3d::Zero();








		//For Inertial Parameter Estimation


	double lambda_factor = 0.005;
	double lambda_factor_2 = 0.001;


	MatrixXd Lambda = lambda_factor*MatrixXd::Identity(6,6);

	int filter_size = 10;
	int filter_size_2 = 8;


	MatrixXd Lambda_2 = lambda_factor_2*MatrixXd::Identity(6,6);

		Lambda_2 << 0.01,  0.0, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
	          0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
	          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
	          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
	          0.0, 0.0, 0.0, 0.0, 0.0, 0.001;



	auto RLS = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);
	auto RLS_2 = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);
	auto RLS_3 = new ParameterEstimation::RecursiveLeastSquare(false,filter_size_2,Lambda_2);
	auto LS = new ParameterEstimation::LeastSquare(false);




	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();


	VectorXd phi_RLS_2 = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS_2 = Matrix3d::Zero();
	Vector3d center_of_mass_RLS_2 = Vector3d::Zero();

	VectorXd phi_RLS_3 = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS_3 = Matrix3d::Zero();
	Vector3d center_of_mass_RLS_3 = Vector3d::Zero();

	VectorXd phi_LS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_LS = Matrix3d::Zero();
	Vector3d center_of_mass_LS = Vector3d::Zero();


	//cutoff frequeny, try frequency of trajectory
	double fc = 20;
	//normalized cutoff frequeency, for butterworth
	double fc_n = fc / (control_freq);

	//time constant, for lowpassfilter
	double tau = 1/(2*M_PI*fc);
	//damping lowpass filer
	double d = 0.7;


	// auto force_filter = new ButterworthFilter(3);
	// force_filter->setCutoffFrequency(fc_n);
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
	low_pass_filter_ft->init(1/control_freq, tau,d);

	auto low_pass_filter_accel = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	low_pass_filter_accel->init(1/control_freq, tau,d);
	Vector3d accel_lp_filtered = Vector3d::Zero();

	auto low_pass_filter_avel = new am2b::SecOrderLowPass<Vector3d>(Vector3d::Zero());
	low_pass_filter_avel->init(1/control_freq, tau,d);

	Vector3d avel_lp_filtered = Vector3d::Zero();
	Vector3d aaccel_lp_filtered = Vector3d::Zero();
	Vector3d avel_butter_filtered = Vector3d::Zero();
	Vector3d accel_butter_filtered = Vector3d::Zero();

		//Read Bias file and write force torque bias in "force_torque_bias" vector
	VectorXd force_moment = VectorXd::Zero(6);
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



	// create timer
	std::chrono::high_resolution_clock::time_point t_start;
	std::chrono::duration<double> t_elapsed;
	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();


		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, force_moment);
			

		// update robot model
		if(flag_simulation)
		{
			redis_client.getEigenMatrixDerived(JOINT_ACCELERATIONS_KEY, robot->_ddq);
			robot->updateModel();
			robot->coriolisForce(coriolis);

			redis_client.getEigenMatrixDerived(LINEAR_ACC_KEY,accel_local);
			redis_client.getEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
			redis_client.getEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
			redis_client.getEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
	 
		}
		else
		{
			robot->updateKinematics();
			redis_client.getEigenMatrixDerived(MASSMATRIX_KEY, robot->_M );
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();

			 redis_client.getEigenMatrixDerived(CORIOLIS_KEY, coriolis);
			force_moment -= force_torque_bias;

		// 	robot->position(current_position_aux, link_name, Vector3d::Zero());
		// 	current_position_aux = R_link.transpose()*current_position_aux;
		//  redis_client.getEigenMatrixDerived(ACCELEROMETER_DATA_KEY, 	accel_aux);
		// 	accel_aux = R_acc_in_ft*accel_aux;
		// 	accel_aux *= 9.81;
		// 	accel_aux += g_local;
		// 	y << current_position_aux, accel_aux;
  //           kalman_filter->update(y);
  //           kf_states = kalman_filter->state();
  //           accel_local << kf_states(6), kf_states(7), kf_states(8);

  //           redis_client.getEigenMatrixDerived(GYROSCOPE_DATA_KEY, avel_aux);
  //           avel_aux = R_acc_in_ft*avel_aux;
  //           avel_aux *= M_PI/180;

  //           q_eff = R_link.transpose();
		// 	q_eff_aux << q_eff.w(), q_eff.vec();

		// 	y_ekf << q_eff_aux, avel_aux;
		// 	extended_kalman_filter-> update(y_ekf);
		// 	ekf_states = extended_kalman_filter->state();
		// 	avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
		// 	aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);
			robot->rotation(R_link,link_name); 
			g_local = R_link.transpose()*robot->_world_gravity;

			redis_client.getEigenMatrixDerived(ACCELEROMETER_DATA_KEY, accel_aux);
			redis_client.getEigenMatrixDerived(GYROSCOPE_DATA_KEY, avel_aux);

			accel_local = R_acc_in_ft*accel_aux;
			accel_local *= 9.81;
			accel_local += g_local;
            avel_local = R_acc_in_ft*avel_aux;
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

		t_start = std::chrono::high_resolution_clock::now();

		t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;

			if(state != GOTO_INITIAL_CONFIG)
			{

				if(controller_counter % 2 == 0)
				{
					if(flag_simulation)
					{
						RLS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
						RLS_2->addData(force_moment, accel_local, avel_local, Vector3d::Zero(), g_local);
					}
					else
					{
						RLS->addData(force_moment_butter_filtered, accel_butter_filtered, avel_butter_filtered, Vector3d::Zero(), g_local);
						RLS_2->addData(force_moment_lp_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);
						RLS_3->addData(force_moment, accel_local, avel_local, Vector3d::Zero(), g_local);
						// LS->addData(force_moment_lp_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);


					}



				// LS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);

				phi_RLS = RLS->getInertialParameterVector();
				center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
				inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);


				phi_RLS_2 = RLS_2->getInertialParameterVector();
				center_of_mass_RLS_2 << phi_RLS_2(1)/phi_RLS_2(0), phi_RLS_2(2)/phi_RLS_2(0), phi_RLS_2(3)/phi_RLS_2(0); 
				inertia_tensor_RLS_2 << phi_RLS_2(4), phi_RLS_2(5), phi_RLS_2(6), phi_RLS_2(5), phi_RLS_2(7), phi_RLS_2(8), phi_RLS_2(6), phi_RLS_2(8), phi_RLS_2(9);
				// cout << "ft: " <<  force_moment.transpose() << " a: " << accel_local.transpose() << " omega: "<<  avel_local.transpose() << " alpha: " << aaccel_local.transpose() << " phi " << phi_RLS_2.transpose() << endl; 


				phi_RLS_3 = RLS_3->getInertialParameterVector();
				center_of_mass_RLS_3 << phi_RLS_3(1)/phi_RLS_3(0), phi_RLS_3(2)/phi_RLS_3(0), phi_RLS_3(3)/phi_RLS_3(0); 
				inertia_tensor_RLS_3 << phi_RLS_3(4), phi_RLS_3(5), phi_RLS_3(6), phi_RLS_3(5), phi_RLS_3(7), phi_RLS_3(8), phi_RLS_3(6), phi_RLS_3(8), phi_RLS_3(9);
				}
			}
		
		

			if(controller_counter%2000==0)
			{
				cout << "1 : current inertial parameters for signals butter filtered, zero angular acceleration " << endl; 
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS << endl;
				
				cout << "2 : current inertial parameters for signals lowpass filtered" << endl; 
		   		cout << "estimated mass: \n" << phi_RLS_2(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS_2.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS_2 << endl;


		   		cout << "3 : current inertial parameters for signals not filtered " << endl; 
		   		cout << "estimated mass: \n" << phi_RLS_3(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS_3.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS_3 << endl;


		  //  		phi_LS = LS->getInertialParameterVector();
				// center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
				// inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);

		  //  		cout << "4 : current inertial parameters for LeastSquare " << endl; 
				// cout << "estimated mass: \n" << phi_LS(0) << endl;
		  // 	  	cout << "estimated center of mass: \n" << 	center_of_mass_LS.transpose() << endl;
		  //  		cout << "estimated Inertia: \n" << inertia_tensor_LS << endl;
			}
		//cout << "Elapsed time Inertial Parameter Estimation: " << t_elapsed.count() << endl;


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
			// cout << config_error.norm() << endl;
			if(config_error.norm() < 0.3)
			// if(config_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				RLS->init();
				RLS_2->init();
				RLS_3->init();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				joint_task->_goal_position = desired_grasping_configuration;
				redis_client.setEigenMatrix(SVH_HAND_POSITION_COMMAND_KEY, hand_pre_grasp);
				cout << "hand_pre_grasp" << hand_pre_grasp.transpose() << endl;
				state = MOVE_TO_OBJECT;

				
			}

		}
		if(state == MOVE_TO_OBJECT)
		{
			// update tasks models

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);


			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;
			VectorXd config_error = desired_grasping_configuration - joint_task->_current_position;
			//cout << config_error.norm() << endl;
			if(config_error.norm() < 0.25)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				
					posori_task->_goal_position =desired_position_above_hand;
					state = MOVE_ABOVE_OBJECT;				
			}

		}

		if(state == MOVE_ABOVE_OBJECT)
		{
			// update tasks models

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques =  joint_task_torques + coriolis + posori_task_torques;



			VectorXd config_error_posori = desired_position_above_hand - posori_task->_current_position;
			//cout << config_error_posori.norm() <<endl;
			if(config_error_posori.norm() < 0.015)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position(2) -= 0.06;
					desired_orientation_above_hand =  posori_task->_current_orientation;
					posori_task->_desired_angular_velocity = Eigen::Vector3d::Zero();


					state = MOVE_DOWN_TO_OBJECT;				
			}

		}

		if(state == MOVE_DOWN_TO_OBJECT)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques =  joint_task_torques + coriolis + posori_task_torques;


			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			//cout << config_error_posori.norm() <<endl;
			if(config_error_posori.norm() < 0.015)
			{
				cout << "current_position move down to obj: " << posori_task->_current_position.transpose() << endl;
				
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position(1) -= 0.035;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state = GRASP;
				
			}

		}

		if(state == GRASP)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;


			n_counter++;
			if(n_counter==wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_2);
			}
			if(n_counter==wait_time*2)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_3);
			}
			if(n_counter==wait_time*3)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_4);
			}
			if(n_counter==wait_time*4)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_6);
			}



			redis_client.getEigenMatrixDerived(SVH_HAND_POSITIONS_KEY, current_position_hand);

			//VectorXd config_error_hand = current_position_hand - hand_grasp_6;
			//cout << config_error_hand.norm() << endl;

			if(n_counter == wait_time*5)
			{
				cout << "current_position grasp: " << posori_task->_current_position.transpose() << endl;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				cout << "_goal_position in grasp: " << posori_task->_goal_position.transpose() << endl;
				desired_orientation_above_hand =  posori_task->_current_orientation;

				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position(2) = 0.6;
				cout << "_goal_position in grasp: " << posori_task->_goal_position.transpose() << endl;
				n_counter = 0;
				state = MOVE_ABOVE_WITH_OBJECT;
				
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

			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;
			//cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.03)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				RLS->init();
				RLS_2->init();
				RLS_3->init();
				// posori_task->_goal_position = V;
				// posori_task->_desired_orientation = desired_orientation_with_obj_up.transpose() * posori_task->_desired_orientation;
				// // posori_task->_desired_orientation = desired_orientation_with_obj_up;
				// cout << " going to CALIB_Up" << endl;
				// cout << "_goal_position in calib_up: " << posori_task->_goal_position.transpose() << endl;
				// cout << "_goal_ori in calib_up: " << posori_task->_desired_orientation << endl;

				// joint_tas


				 
				// joint_task->_goal_position(6) -= M_PI/5;
				posori_task->_goal_position = Vector3d(.4,0.3,0.4);
				posori_task->_desired_orientation = desired_orientation_z_down;
				state = MOVE_SIN;
				
			}
		}


		if(state == MOVE_DOWN_WITH_OBJECT)

		{
			// update tasks models

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
							cout << "_goal_position MOVE_DOWN_WITH_OBJECT " << posori_task->_goal_position.transpose() << endl;
														cout << "_cuurr MOVE_DOWN_WITH_OBJECT " << posori_task->_current_position.transpose() << endl;
				cout << "goal_orentation MOVE_DOWN_WITH_OBJECT " << posori_task->_desired_orientation << endl;

				cout << "goal_orentation MOVE_DOWN_WITH_OBJECT " << posori_task->_current_orientation << endl;



			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			pos_error = posori_task->_goal_position - posori_task->_current_position;

			if(pos_error.norm() < 0.02)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				joint_task->_goal_position(6) += M_PI/6;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state = LET_GO_OF_OBJECT;
				n_counter= 0;
			}
			

		}


		if(state == LET_GO_OF_OBJECT)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			//N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
cout<< "let go" << endl;

			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			n_counter++;
			if (n_counter == wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_1);
			}

			if (n_counter == 2*wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_2);
			}
			if (n_counter == 3*wait_time)
			{
			redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_home);
			}
				

			

			if(n_counter== 3*wait_time)
			{
				n_counter = 0;

				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position(1) += 0.1;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				cout<< "move back" << endl;

				state = MOVE_BACK;
				
			}

		}

		if(state == MOVE_BACK)

		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			VectorXd pos_error = posori_task->_goal_position - posori_task->_current_position;

			// cout << posori_task->_goal_position.transpose()  << " " << posori_task->_current_position.transpose()<< endl;
			if(pos_error.norm() < 0.03)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				joint_task->_goal_position = desired_initial_configuration;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
								cout<< "initial" << endl;

				state = GO_BACK_TO_INITIAL_CONFIG;
				
			}
			

		}

		if(state == GO_BACK_TO_INITIAL_CONFIG)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->computeTorques(joint_task_torques);

			command_torques =  joint_task_torques + coriolis;


		}

		if(state == CALIB_UP)
		{	
			// update tasks models
			N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = joint_task_torques + coriolis;

 		// 	double gamma =  M_PI/5.5/1000* n_counter;
 		// 	cout << "gamma" << gamma << "n_counter " << n_counter << endl;
 		// 	desired_orientation_with_obj_up = Matrix3d::Zero();

			// desired_orientation_with_obj_up << 1 ,0, 0,
 		// 								0  ,cos(gamma),- sin(gamma), 
 		// 								0 , sin(gamma), cos(gamma);
 		// 	n_counter++;

 			// posori_task->_desired_orientation = desired_orientation_with_obj_up.transpose() * posori_task->_current_orientation;
			// Sai2Model::orientationError(ori_error, Matrix3d::Identity(), posori_task->_current_orientation);
			// pos_error = desired_position_move_with_obj_up - posori_task->_current_position;
			VectorXd config_error = calib_up - joint_task->_current_position;

			// cout << ori_error.norm() << endl;
			if(config_error.norm() < 0.35)

			// if(gamma > M_PI/4)
			{	


				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				// posori_task-> _goal_position = desired_position_letgo_hand - Vector3d(0.0, -0.1, -0.2) ;
				cout << "_goal_position in in CALIB_up: " << posori_task->_current_position.transpose() << endl;
				cout << "_orentation in in CALIB_up: " << posori_task->_current_orientation.transpose() << endl;

				cout << " going to CALIB_down" << endl;

				// posori_task->_goal_position = desired_position_move_with_obj_down;
				// posori_task->_desired_orientation = desired_orientation_with_obj_down.transpose() * posori_task->_current_orientation;
				// posori_task->_desired_orientation = desired_orientation_with_obj_down;
				cout << "_goal_position in calib_down: " << posori_task->_goal_position.transpose() << endl;
				cout << "_goal_ori in calib_down: " << posori_task->_desired_orientation << endl;

				joint_task->_goal_position = calib_down;
				// posori_task->enableVelocitySaturation(vel_sat, avel_sat);


				state = CALIB_DOWN;

				n_counter = 0;

				
			}

		}

		if(state == CALIB_DOWN)
		{	
			// update tasks models
			N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

 		// 	double gamma =  M_PI/5.5/1000* n_counter;
 		// 	cout << "gamma" << gamma << "n_counter " << n_counter << endl;
 		// 	Matrix3d desired_orientation_with_obj_down = Matrix3d::Zero();

			// desired_orientation_with_obj_down << 1 ,0, 0,
 		// 								0  ,cos(gamma),-sin(gamma), 
 		// 								0 , sin(gamma), cos(gamma);

 			// posori_task->_desired_orientation = desired_orientation_with_obj_down.transpose() * posori_task->_current_orientation;
 			// posori_task->_desired_angular_velocity = Eigen::Vector3d::Zero();

			command_torques = joint_task_torques + coriolis;
			// Matrix3d ori_des = Matrix3d::Zero();

			// Sai2Model::orientationError(ori_error, desired_orientation_z_down, posori_task->_current_orientation);
			// Vector3d ori_error = Vector3d(0.0, -1.0, 0.0) - posori_task->_current_orientation.col(2);
			// n_counter++;
			// posori_error =  ori_error;
			VectorXd config_error = calib_up - joint_task->_current_position;

			if(config_error.norm() < 0.35)
			{
			// cout << ori_z_error.norm() << endl << "zaxis: " << posori_task->_current_orientation.col(2) << endl;
			// if(gamma>2/(3*M_PI))
			// {
				// if (ori_error.norm() < 0.15)
				// {
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					cout << "_goal_position in in CALIB_Down " << posori_task->_current_position.transpose() << endl;
					cout << "_orentation in in CALIB_DOWN: " << posori_task->_current_orientation << endl;
					cout << "_orentation indes: " << desired_orientation_above_hand << endl;



					//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					n_counter = 0;

					posori_task->_goal_position = desired_position_letgo_hand;
					posori_task->_desired_orientation = desired_orientation_above_hand;
					state = MOVE_DOWN_WITH_OBJECT;
				}
				}
			// }

			if(state = MOVE_SIN)
			{
							// update tasks models
				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				pos_task->updateTaskModel(N_prec);
				N_prec = pos_task->_N;
				joint_task->updateTaskModel(N_prec);


			
			robot->position(current_position, posori_task->_link_name, pos_in_link);
			double circle_radius = 0.3;
			double circle_freq = 0.3;
			// posori_task2->_desired_position = current_position + circle_radius * Eigen::Vector3d(sin(2*M_PI*circle_freq*timer.elapsedTime()), cos(2*M_PI*circle_freq*timer.elapsedTime()), 0.0);
			// posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(cos(2*M_PI*circle_freq*timer.elapsedTime()), -sin(2*M_PI*circle_freq*timer.elapsedTime()),0.0);
			pos_task->_goal_position = current_position + circle_radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()));
			pos_task->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, 0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()));

				posori_task->computeTorques(posori_task_torques);
				pos_task->computeTorques(pos_task_torques);
				joint_task->computeTorques(joint_task_torques);

				command_torques = pos_task_torques + posori_task_torques + joint_task_torques;
				pos_error = posori_task->_goal_position - posori_task->_current_position;

				if(pos_error.norm()< 0.05)
				{
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					pos_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);

					posori_task->_goal_position = desired_position_letgo_hand;
					posori_task->_desired_orientation = desired_orientation_above_hand;
					state = MOVE_DOWN_WITH_OBJECT;

				}



			}






		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_LP_KEY, accel_lp_filtered);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_LP_KEY, avel_lp_filtered);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_LP_KEY, aaccel_lp_filtered);

		redis_client.setEigenMatrixDerived(LINEAR_ACC_BUTTER_KEY, accel_butter_filtered);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_BUTTER_KEY, avel_butter_filtered);

		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_BUTTER_KEY, force_moment_butter_filtered);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_LP_KEY, force_moment_lp_filtered);



		redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_aux);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, force_moment);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_BUTTER_KEY, phi_RLS);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_LP_KEY, phi_RLS_2);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi_RLS_3);



		//offline processing
		// if(state !=GOTO_INITIAL_CONFIG)
		// {
		// 	redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_LOCAL_KEY, accel_aux);
		// 	redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_aux);
		// 	redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, force_moment);
		// 	redis_client.setEigenMatrixDerived(QUATERNION_KEY, q_eff_aux);
		// 	redis_client.setEigenMatrixDerived(POSITION_KEY, current_position_aux);	
		// }




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