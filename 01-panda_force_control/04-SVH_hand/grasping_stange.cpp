#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "parameter_estimation/RecursiveLeastSquare.h"
#include "parameter_estimation/LeastSquare.h"
#include "filters/SecOrderLowPass.hpp"
#include "filters/ButterworthFilter.h"





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

// const bool flag_simulation = true;
const bool flag_simulation = false;

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

string INERTIAL_PARAMS_LP_KEY ;
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
#define  REST 						11

#define  MOVE_AWAY_WITH_OBJECT		10




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
		INERTIAL_PARAMS_LP_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::lowpass";
		INERTIAL_PARAMS_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::butter";
		// QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
		// POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";

		SVH_HAND_POSITION_COMMAND_KEY ="sai2::SVHHand_Left::position_command";
		SVH_HAND_GRASP_FLAG_KEY ="sai2::SVHHand_Left::grasp_type_flag";
		SVH_HAND_POSITIONS_KEY = "sai2::SVHHand_Left::position"; 

		
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

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	//Controllers
	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	Vector3d avel_sat = Vector3d(M_PI/5.5, M_PI/5.5, M_PI/5.5);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 75.0;
	posori_task->_kv_pos = 2.3*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 52.0;
	posori_task->_kv_ori = 2.1*sqrt(posori_task->_kp_ori);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// position controller for angular motion


	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/9.5;
	// joint_task->_max_velocity = 0;
	joint_task->_kp = 70.0;
	joint_task->_kv = 2 * sqrt(joint_task->_kp);
	joint_task->_ki = 2;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	VectorXd desired_final_configuration = VectorXd::Zero(dof);
	VectorXd desired_grasping_configuration = VectorXd::Zero(dof);
	VectorXd desired_letgo_configuration = VectorXd::Zero(dof);


	
	
	

	
	desired_initial_configuration << -1.75804,-0.0814604,1.56435,-1.6621,0.114415,1.78799,-0.593858;
	desired_grasping_configuration << -1.77017,-1.66498,1.70429,-2.13006,0.176574,2.04308,-0.593889;
	desired_letgo_configuration << -1.77017,-1.66498,1.70429,-2.13006,0.176574,2.04308,-0.593889;

	desired_final_configuration << 0.0,-0.2,-0.3,-1.7,-0.06,1.5,-2.0;
	// desired_initial_configuration << 0.0,0.0,-0.3,-1.7,0.0,1.5,-2.0;
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
	double thumb_flex_grasp_2 	= 0.37;

	double thumb_oppo_grasp 	 = 0.99;
	double thumb_oppo_grasp_2 	 = 0.99;

	double index_dist_grasp 	 = 0.9;
	double index_dist_grasp_2 	 = .4;

	double index_prox_grasp  	 = 0.7;
	double index_prox_grasp_2 	 = 0.44; 

	double middle_dist_grasp  	 = 0.55;
	double middle_dist_grasp_2  = 0.9;

	double middle_prox_grasp	 = 0.8;
	double middle_prox_grasp_2	 = 0.7;

	double ring_grasp 		 	 = 0.34;
	double ring_grasp_2 		 = 0.44;

	double pinky_grasp		     = 0.4;

	double pinky_grasp_2		   = 0.46;

	double finger_spread_grasp  = 0.35;
	double finger_spread_grasp_2  = 0.53;


	double thumb_flex_aux		 = 0.15;
	double index_prox_aux        = 0.3;
	double index_dist_aux		 = 0.5;
	double middle_prox_aux		 = 0.3;

	double thumb_flex_aux_2		 = 0.15;


	// hand_SECOND_pre_grasp_2 << 0, thumb_oppo_grasp, 0, 0,0, 0, 0, 0, finger_spread_grasp;


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
	hand_pre_grasp <<0, thumb_oppo_grasp_2, 0, 0,0, 0, 0, 0, finger_spread_grasp_2;
	hand_grasp_2 << 0.150000,0.989725,0.400000,0.5000,0.30000,0.6,0.0,0.300000,0.529991;
	hand_grasp_3 << 0.330000,0.98,0.800000,0.5000,0.8500,0.7,.45,0.460000,0.529991;
	hand_let_go_1 << 0.330000,0.989725,0.170000,0.5000,0.9000,0.7,.02,0.00600,0.529991;
	VectorXd hand_grasp_stange = VectorXd::Zero(9);

	int n_counter;
	int wait_time = 800;
	Vector3d desired_position_above_hand = Vector3d(0.54, -0.15,  0.245);
	Vector3d desired_position_letgo_hand = Vector3d(0.54, -0.15, 0.25);
	Vector3d desired_position_away_ob = Vector3d(0.6,0.2,0.6);
	Matrix3d desired_orientation_above_hand = Matrix3d::Zero();


	//cutoff frequeny, try frequency of trajectory
	double fc = 20;
	//normalized cutoff frequeency, for butterworth
	double fc_n = fc / (control_freq);

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
	// auto LS = new ParameterEstimation::LeastSquare(false);




	VectorXd phi_RLS = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS = Matrix3d::Zero();
	Vector3d center_of_mass_RLS = Vector3d::Zero();


	VectorXd phi_RLS_2 = VectorXd::Zero(10); //inertial parameter vector
	Matrix3d inertia_tensor_RLS_2 = Matrix3d::Zero();
	Vector3d center_of_mass_RLS_2 = Vector3d::Zero();



	// VectorXd phi_LS = VectorXd::Zero(10); //inertial parameter vector
	// Matrix3d inertia_tensor_LS = Matrix3d::Zero();
	// Vector3d center_of_mass_LS = Vector3d::Zero();



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
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);
			
		// robot->rotation(R_link,link_name); 
		// g_local = R_link.transpose()*robot->_world_gravity;
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
			robot->rotation(R_link,link_name); 
			g_local = R_link.transpose()*robot->_world_gravity;
			// g_local = redis_client.getEigenMatrixJSON(ROBOT_GRAVITY_KEY);
			// g_local = R_link.transpose
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

			// robot->position(current_position_aux, link_name, Vector3d::Zero());
			// current_position_aux = R_link.transpose()*current_position_aux;
			// accel_aux = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			// accel_aux = R_acc_in_ft*accel_aux;
			// accel_aux *= 9.81;
			// accel_aux += g_local;
			// y << current_position_aux, accel_aux;
   //          kalman_filter->update(y);
   //          kf_states = kalman_filter->state();
   //          accel_local << kf_states(6), kf_states(7), kf_states(8);

   //          avel_aux = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
   //          avel_aux = R_acc_in_ft*avel_aux;
   //          avel_aux *= M_PI/180;

   //          q_eff = R_link.transpose();
			// q_eff_aux << q_eff.w(), q_eff.vec();

			// y_ekf << q_eff_aux, avel_aux;
			// extended_kalman_filter-> update(y_ekf);
			// ekf_states = extended_kalman_filter->state();
			// avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
			// aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);
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
				}

				else
				{
					RLS->addData(force_moment_butter_filtered, accel_butter_filtered, avel_butter_filtered, Vector3d::Zero(), g_local);
					RLS_2->addData(force_moment_lp_filtered, accel_lp_filtered, avel_lp_filtered, aaccel_lp_filtered, g_local);					
				}

			// LS->addData(force_moment, accel_local, avel_local, aaccel_local, g_local);
			phi_RLS = RLS->getInertialParameterVector();
			center_of_mass_RLS << phi_RLS(1)/phi_RLS(0), phi_RLS(2)/phi_RLS(0), phi_RLS(3)/phi_RLS(0); 
			inertia_tensor_RLS << phi_RLS(4), phi_RLS(5), phi_RLS(6), phi_RLS(5), phi_RLS(7), phi_RLS(8), phi_RLS(6), phi_RLS(8), phi_RLS(9);


			phi_RLS_2 = RLS_2->getInertialParameterVector();
			center_of_mass_RLS_2 << phi_RLS_2(1)/phi_RLS_2(0), phi_RLS_2(2)/phi_RLS_2(0), phi_RLS_2(3)/phi_RLS_2(0); 
			inertia_tensor_RLS_2 << phi_RLS_2(4), phi_RLS_2(5), phi_RLS_2(6), phi_RLS_2(5), phi_RLS_2(7), phi_RLS_2(8), phi_RLS_2(6), phi_RLS_2(8), phi_RLS_2(9);
			// cout << "ft: " <<  force_moment.transpose() << " a: " << accel_local.transpose() << " omega: "<<  avel_local.transpose() << " alpha: " << aaccel_local.transpose() << " phi " << phi_RLS.transpose() << endl; 


			}

			if(controller_counter%700==0)
			{
				cout << "1 : current inertial parameters for signals butter filtered, zero angular acceleration " << endl; 
				cout << "estimated mass: \n" << phi_RLS(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS << endl;
				
				cout << "2 : current inertial parameters for signals lowpass filtered" << endl; 
		   		cout << "estimated mass: \n" << phi_RLS_2(0) << endl;
		  	  	cout << "estimated center of mass: \n" << 	center_of_mass_RLS_2.transpose() << endl;
		   		cout << "estimated Inertia: \n" << inertia_tensor_RLS_2 << endl;



		 //   		phi_LS = LS->getInertialParameterVector();
			// center_of_mass_LS << phi_LS(1)/phi_LS(0), phi_LS(2)/phi_LS(0), phi_LS(3)/phi_LS(0); 
			// inertia_tensor_LS << phi_LS(4), phi_LS(5), phi_LS(6), phi_LS(5), phi_LS(7), phi_LS(8), phi_LS(6), phi_LS(8), phi_LS(9);

			// 				cout << "estimated mass: \n" << phi_LS(0) << endl;
		 //  	  	cout << "estimated center of mass: \n" << 	center_of_mass_LS.transpose() << endl;
		 //   		cout << "estimated Inertia: \n" << inertia_tensor_LS << endl;
			}
		}
		

		//cout << "Elapsed time Inertial Parameter Estimation: " << t_elapsed.count() << endl;

		//robot->position(current_position_aux,link_name,Vector3d::Zero());
		if(controller_counter%1000)
		{
			// cout << "current_position: " << posori_task->_current_position.transpose() << endl;
			// cout << "current_orientation: " << posori_task->_current_orientation.transpose() << endl;
			// cout << "current_state: " << state << endl;
		}
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
			if(config_error.norm() < 0.2)
			// if(config_error.norm() < 0.05)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				joint_task->_goal_position = desired_grasping_configuration;
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_pre_grasp);
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
			// cout << config_error.norm() << endl;
			if(config_error.norm() < 0.1)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(vel);

				
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
			if(config_error_posori.norm() < 0.09)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();

					cout << "current_position move above obj: " << posori_task->_current_position.transpose() << endl;
					cout << "current_orientation move above obj: " << posori_task->_current_orientation.transpose() << endl;


					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position(2) += 0.063;
					
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
			// cout << config_error_posori.norm() <<endl;
			if(config_error_posori.norm() < 0.06)
			{
				cout << "current_position move down to obj: " << posori_task->_current_position.transpose() << endl;
				cout << "current_orientation move down obj: " << posori_task->_current_orientation.transpose() << endl;

				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				// posori_task->_goal_position(2) += 0.025;
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
			if(n_counter==2*wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_2);
			}
			if(n_counter==wait_time*4)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_3);
			}
			// if(n_counter==wait_time*3)
			// {
			// 	redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_4);
			// }
			// if(n_counter==wait_time*4)
			// {
			// 	redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_6);
			// }
// /


			redis_client.getEigenMatrixDerived(SVH_HAND_POSITIONS_KEY, current_position_hand);

			//VectorXd config_error_hand = current_position_hand - hand_grasp_6;
			//cout << config_error_hand.norm() << endl;

			if(n_counter == wait_time*5)
			{
				// cout << "current_position grasp: " << posori_task->_current_position.transpose() << endl;
				robot->rotation(desired_orientation_above_hand, link_name);
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				// cout << "_goal_position in grasp: " << posori_task->_goal_position.transpose() << endl;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position(2) += 0.25;
				posori_task->_goal_position(1) += 0.25;
				posori_task->_goal_position(0) += 0.15;
				// cout << "_goal_position in grasp: " << posori_task->_goal_position.transpose() << endl;
				n_counter = 0;
				// joint_task->_goal_position(1) += 0.5*M_PI;
				// joint_task->_goal_position(0) += M_PI/3;
				cout << "estimation with object" << endl;
				// state = MOVE_ABOVE_WITH_OBJECT;
				state = MOVE_AWAY_WITH_OBJECT;
				
			}

		}


		if(state == MOVE_ABOVE_WITH_OBJECT)
		{
			// // update tasks models
			N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			// cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.15)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position =  desired_position_letgo_hand;
				 
				// cout << "_goal_position in MOVE_ABOVE_WITH_OBJECT: " << posori_task->_goal_position.transpose() << endl;
				// joint_task->_goal_position(6) -= M_PI/5;
				// posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE_DOWN_WITH_OBJECT;
				
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


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;

			// cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.015)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position(2) -= 0.02;
				// joint_task->_goal_position(6) += M_PI/6;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state = LET_GO_OF_OBJECT;
				
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


			command_torques = posori_task_torques+ joint_task_torques + coriolis;

			n_counter++;
			if (n_counter == 2*wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_1);
			}

			if (n_counter == 3*wait_time)
			{
				// redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_2);
			}
			

			if(n_counter== 3*wait_time)
			{
				n_counter = 0;

				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position(2) -= 0.03;
				posori_task->_goal_position(1) += 0.15;
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				cout << "estimation without object" << endl;
				
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

			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;

			// cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				joint_task->_goal_position = desired_initial_configuration;
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_home);

				state = REST;
				
			}
			

		}

		// if(state == PRE)

		// {
		// 	// update tasks models
		// 	N_prec.setIdentity();
		// 	posori_task->updateTaskModel(N_prec);
		// 	N_prec = posori_task->_N;
		// 	joint_task->updateTaskModel(N_prec);

		// 	posori_task->computeTorques(posori_task_torques);
		// 	joint_task->computeTorques(joint_task_torques);


		// 	command_torques = posori_task_torques+ joint_task_torques + coriolis;

		// 	VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;

		// 	// cout << config_error.norm() <<endl;
		// 	if(config_error.norm() < 0.02)
		// 	{
		// 		joint_task->reInitializeTask();
		// 		posori_task->reInitializeTask();
		// 		// joint_task->_goal_position = desired_final_configuration;
		// 		redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_home);
		// 		posori_task->enableVelocitySaturation(vel_sat, avel_sat);
		// 		state = GO_BACK_TO_INITIAL_CONFIG;
				
		// 	}
			

		// }

		if(state == REST)
		{
			N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;


		}

		if(state == MOVE_AWAY_WITH_OBJECT)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques+ joint_task_torques + coriolis;


			VectorXd config_error = posori_task->_goal_position - posori_task->_current_position;
			n_counter ++;
			// cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				// posori_task->_goal_position = desired_position_letgo_hand + Vector3d(0.0,0.0,0.1);
				joint_task->_goal_position= desired_letgo_configuration;
				// cout << "MBO" << endl;
				n_counter=0;
				// posori_task->disableVelocitySaturation();
				state = MOVE_ABOVE_WITH_OBJECT;
				
			}

		}





		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		// redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		// redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		// redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		// redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_LP_KEY, phi_RLS_2);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_BUTTER_KEY, phi_RLS);



		//offline processing
		if(state !=GOTO_INITIAL_CONFIG)
		{
			// redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_LOCAL_KEY, accel_aux);
			// redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_aux);
			// redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, force_moment);
			// redis_client.setEigenMatrixDerived(QUATERNION_KEY, q_eff_aux);
			// redis_client.setEigenMatrixDerived(POSITION_KEY, current_position_aux);	
		}




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