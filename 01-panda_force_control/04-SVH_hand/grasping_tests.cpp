#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"


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
std::string SVH_HAND_POSITION_COMMAND_KEY;
std::string SVH_HAND_GRASP_FLAG_KEY;

// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
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

/* Data Matrix Force/Torque virtual A_t based on 
 *"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 */
Eigen::MatrixXd GetDataMatrixFT(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local);
/* Data Matrix Force/Torque virtual A_t based on 
 * "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
*/
Eigen::MatrixXd computeSigma(Eigen::MatrixXd K, Eigen::MatrixXd Sigma, Eigen::MatrixXd A);
Eigen::MatrixXd computeK(Eigen::MatrixXd Sigma, Eigen::MatrixXd A, Eigen::MatrixXd Lambda);

void computeInertial(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma); 



int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

		LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::accel";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";
		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";

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
		EE_FORCE_SENSOR_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";
		QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
		POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";

		SVH_HAND_POSITION_COMMAND_KEY ="sai2::SVHHand_Left::position_command";
		SVH_HAND_GRASP_FLAG_KEY ="get sai2::SVHHand_Left::grasp_type_flag";
		SVH_HAND_POSITIONS_KEY = "sai2::SVHHand_Left::position"; 

		
	}



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
	Vector3d avel_sat = Vector3d(M_PI/7.5, M_PI/7.5, M_PI/7.5);
	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 65.0;
	posori_task->_kv_pos = 2.2*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_ori);
	// posori_task->_velocity_saturation = true;
	// posori_task->_linear_saturation_velocity = vel_sat;
	// posori_task->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// position controller for angular motion


	//joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/7.5;
	// joint_task->_max_velocity = 0;
	joint_task->_kp = 41.0;
	joint_task->_kv = 2.1 * sqrt(joint_task->_kp);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	VectorXd desired_grasping_configuration = VectorXd::Zero(dof);
	VectorXd desired_letgo_configuration = VectorXd::Zero(dof);
	
	
	

	
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
    auto kalman_filter = new KalmanFilter(dt, A, C, Q, R, P);
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

  	auto extended_kalman_filter = new QuaternionBasedEKF( dt, C_ekf, Q_ekf, R_ekf, P_ekf);

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
	hand_let_go_2 << thumb_flex_home, thumb_oppo_home, index_dist_home, index_prox_home, middle_dist_home, middle_prox_home, ring_home, pinky_home, finger_spread_home;
	//cout << "hand_pre_grasp" << hand_pre_grasp.transpose() << endl;
	int n_counter;
	int wait_time = 1000;
	Vector3d desired_position_above_hand = Vector3d(0.64, -0.23,  0.4);
	Vector3d desired_position_letgo_hand = Vector3d(0.64, -0.23, 0.38);
	Matrix3d desired_orientation_above_hand = Matrix3d::Zero();
	desired_orientation_above_hand << -0.8  , 0.2  , -0.6,
   										 0.57  ,  -0.14,   -0.8,
  										-0.24,    -0.97, -0;


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

			robot->position(current_position_aux, link_name, Vector3d::Zero());
			current_position_aux = R_link.transpose()*current_position_aux;
			accel_aux = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
			accel_aux = R_acc_in_ft*accel_aux;
			accel_aux *= 9.81;
			accel_aux += g_local;
			y << current_position_aux, accel_aux;
            kalman_filter->update(y);
            kf_states = kalman_filter->state();
            accel_local << kf_states(6), kf_states(7), kf_states(8);

            avel_aux = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
            avel_aux = R_acc_in_ft*avel_aux;
            avel_aux *= M_PI/180;

            q_eff = R_link.transpose();
			q_eff_aux << q_eff.w(), q_eff.vec();

			y_ekf << q_eff_aux, avel_aux;
			extended_kalman_filter-> update(y_ekf);
			ekf_states = extended_kalman_filter->state();
			avel_local << ekf_states(4), ekf_states(5), ekf_states(6);
			aaccel_local << ekf_states(7), ekf_states(8), ekf_states(9);
		}

		t_start = std::chrono::high_resolution_clock::now();
		A_data = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
		computeInertial(n_measurements, force_moment, A_data, phi,  Sigma);
		n_measurements++;
		t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;
		//cout << "Elapsed time Inertial Parameter Estimation: " << t_elapsed.count() << endl;

		//robot->position(current_position_aux,link_name,Vector3d::Zero());
		if(controller_counter%1000)
		{
			cout << "current_position: " << posori_task->_current_position.transpose() << endl;
			//cout << "current_orientation: " << posori_task->_current_orientation.transpose() << endl;
			cout << "current_state: " << state << endl;
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
			cout << config_error.norm() << endl;
			if(config_error.norm() < 0.3)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
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
			cout << config_error.norm() << endl;
			if(config_error.norm() < 0.3)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				
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
			cout << config_error_posori.norm() <<endl;
			if(config_error_posori.norm() < 0.03)
			{	
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task->_goal_position(2) -= 0.13;
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
			cout << config_error_posori.norm() <<endl;
			if(config_error_posori.norm() < 0.03)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
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
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_1);
			}
			if(n_counter==wait_time*2)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_2);
			}
			if(n_counter==wait_time*3)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_3);
			}
			if(n_counter==wait_time*4)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_4);
			}
			if(n_counter==wait_time*5)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_5);
			}
			if(n_counter==wait_time*6)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_6);
			}

			redis_client.getEigenMatrixDerived(SVH_HAND_POSITIONS_KEY, current_position_hand);

			//VectorXd config_error_hand = current_position_hand - hand_grasp_6;
			//cout << config_error_hand.norm() << endl;

			if(n_counter == wait_time*7)
			{

				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task->_goal_position(2) += 0.5;
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
			cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.1)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position =  desired_position_letgo_hand;
				joint_task->_goal_position(6) -= M_PI/5;
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state == MOVE_DOWN_WITH_OBJECT;
				
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

			VectorXd config_error = desired_position_above_hand - posori_task->_current_position;

			cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state == LET_GO_OF_OBJECT;
				
			}
			

		}


		if(state == LET_GO_OF_OBJECT)
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
			if (n_counter == wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_1);
			}

			if (n_counter == 2*wait_time)
			{
				redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_let_go_2);
			}
			

			if(n_counter== 3*wait_time)
			{
				n_counter = 0;

				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->_goal_position(1) -=0.15;
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				//redis_client.setEigenMatrixDerived(SVH_HAND_POSITION_COMMAND_KEY, hand_grasp_5);
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

			cout << config_error.norm() <<endl;
			if(config_error.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				joint_task->_goal_position = desired_initial_configuration;
				//posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				state == GO_BACK_TO_INITIAL_CONFIG;
				
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





		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi);

		//offline processing
		if(state !=GOTO_INITIAL_CONFIG)
		{
			redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_LOCAL_KEY, accel_aux);
			redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_aux);
			redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, force_moment);
			redis_client.setEigenMatrixDerived(QUATERNION_KEY, q_eff_aux);
			redis_client.setEigenMatrixDerived(POSITION_KEY, current_position_aux);	
		}




		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    inertia_tensor << phi(4), phi(5), phi(6), phi(5), phi(7), phi(8), phi(6), phi(8), phi(9); 
	center_of_mass << phi(1)/phi(0), phi(2)/phi(0), phi(3)/phi(0); 
    std::cout << "estimated mass" << phi(0) << "\n";
    std::cout << "estimated center of mass" << 	center_of_mass.transpose() << "\n";
    std::cout << "estimated Inertia" << inertia_tensor << "\n";

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;

}

// - Data Matrix A_t Full based on "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008
// - accel_local: object linear acceleration in sensor frame
// - aaccel_local: object angular acceleration in sensor frame
// - avel_local: object angular velocity in sensor frame
// - g_local: gravity vector in sensor frame
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


void computeInertial(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma)
{
	if (n_measurements == 0)
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