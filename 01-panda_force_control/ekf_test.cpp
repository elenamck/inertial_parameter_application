#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"
#include "filters/ButterworthFilter.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"
#include "filters/SavitzkyGolayFilter.h"

#include <fstream>
#include <iostream>
#include <string>
#include <tinyxml2.h>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

#define  GOTO_INITIAL_CONFIG 		 0
#define  MOVE_TO_OBJECT        		 1
#define  MOVE 						 2
#define  MOVE2 						 3
#define  HAND_OVER 					 4

unsigned long long controller_counter = 0;

// const bool flag_simulation = true;
const bool flag_simulation = false;

const bool inertia_regularization = true;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string LINEAR_ACCELERATION_LOCAL_KEY;
std::string ANGULAR_VELOCITY_LOCAL_KEY;
std::string ANGULAR_ACCELERATION_LOCAL_KEY;
std::string POSITION_KEY;
std::string LINEAR_VELOCITY_KEY;
std::string KALMAN_FILTER_POS_KEY;
std::string KALMAN_FILTER_VEL_KEY;
std::string KALMAN_FILTER_ACC_KEY;
std::string LOCAL_GRAVITY_KEY;
std::string LINEAR_ACCELERATION_CORRECTED_RIGHT_FRAME_KEY;

std::string ANGULAR_VEL_ESTIMATE_KEY;
std::string ANGULAR_ACC_ESTIMATE_KEY;
std::string QUATERNION_KEY;
std::string QUATERNION_ESTIMATE_KEY;

std::string ANGULAR_ACCELERATION_KINEMATICS_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;
std::string ACCELEROMETER_DATA_KEY;
std::string GYROSCOPE_DATA_KEY;
std::string LINEAR_ACCELERATION_CORRECTED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

//delete after checking sensor signal
std::string LINEAR_ACCELERATION_KINEMATICS_KEY;
std::string ANGULAR_VELOCITY_KINEMATICS_KEY;

int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
		LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::accel";
		ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
		ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";
		POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
		LINEAR_VELOCITY_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
		KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::Panda::KF::position";
		KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::Panda::KF::velocity";
		KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::Panda::KF::acceleration";

		ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::angular_vel";
		ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::angular_acc";
		QUATERNION_KEY = "sai2::DemoApplication::simulation::Panda::controller::logging::quaternion";
		QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::quaternion";

		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";




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
		ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";        // in local frame
		GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";            // in local frame
		LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::accel";
		ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::avel";
		ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::aaccel";
		POSITION_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::pos";
		LINEAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
		KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::position";
		KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
		KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::acceleration";

		LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::FrankaPanda::Clyde::g_local";
		LINEAR_ACCELERATION_CORRECTED_KEY =  "sai2::3spaceSensor::data::linear_acceleration_corrected";
		LINEAR_ACCELERATION_CORRECTED_RIGHT_FRAME_KEY ="sai2::DemoApplication::FrankaPanda::Clyde::linear_acceleration_corrected_right_frame";


		////////////////////////////////////////////DELETE AFTER CHECKING//////////////////////
		LINEAR_ACCELERATION_KINEMATICS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel_kinematics";
		ANGULAR_VELOCITY_KINEMATICS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::avel_kinematics";
		ANGULAR_ACCELERATION_KINEMATICS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::aaccel_kinematics";

		////////////////////////////TEST EKF/////////////////////////////////////
		ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_vel";
		ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_acc";
		QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::logging::quaternion";
		QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::quaternion";


	}
		// redis keys

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
	///            Sensor variables            /////
	////////////////////////////////////////////////
	//to compare sensor data with kinematics
	Vector3d accel_kinematics = Vector3d::Zero();
	Vector3d avel_kinematics = Vector3d::Zero();
	Vector3d aaccel_kinematics = Vector3d::Zero();
	Vector3d g_local = Vector3d::Zero();
	//Accelerometer and Gyroscope
	Vector3d accel = Vector3d::Zero(); //object linear acceleration in base frame
	Vector3d avel = Vector3d::Zero(); //object angular velocity in base frame
	Vector3d aaccel = Vector3d::Zero(); // object angular acceleration in base frame
	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d accel_sensor = Vector3d::Zero();
	Vector3d accel_gravity_removed = Vector3d::Zero();
	//Rotation Matrix Accelerometer in FT Sensor frame
	Matrix3d R_acc_in_ft = Matrix3d::Zero();
	R_acc_in_ft <<  0,  0,  1,
				    1,  0,  0,
				    0,  1,  0;

	//FT Bias
	VectorXd force_torque_bias = VectorXd::Zero(6); //FT Bias
	VectorXd force_moment= VectorXd::Zero(6);
	ifstream bias;
	bias.open("FT_data1.txt");
	if (!bias)  
	{                     // if it does not work
        std::cout << "Can't open Data!\n";
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

	int counter = 0;

	ButterworthFilter* filter = new ButterworthFilter(3, 1000, 0.5);
	Vector3d avel_local_unfiltered = Vector3d::Zero();
	Vector3d avel_local_last = Vector3d::Zero();





	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.1);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.11;
	// posori_task->_max_velocity = 0;

	// posori_task->_kp_pos = 100.0;
	// posori_task->_kv_pos = 20.0;
	// posori_task->_kp_ori = 100.0;
	// posori_task->_kv_ori = 20.0;
	posori_task->_kp_pos = 60.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_ori);


	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = Eigen::Vector3d(0.2,0.2,0.2);
	posori_task->_angular_saturation_velocity = Eigen::Vector3d(M_PI/4.0, M_PI/4.0, M_PI/4.0);
	//posori_task->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
	// posori_task->_velocity_saturation = true;
	// posori_task->_velocity_saturation = false;
	// posori_task->_linear_saturation_velocity = Eigen::Vector3d(0.13,0.13,0.13);
	// posori_task->_angular_saturation_velocity = Eigen::Vector3d(M_PI/4, M_PI/4, M_PI/4);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	Matrix3d initial_orientation = Matrix3d::Zero();

	// joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = M_PI/7.5;
	// joint_task->_max_velocity = 0;
	joint_task->_kp = 41.0;
	joint_task->_kv = 2.1 * sqrt(joint_task->_kp);

	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	//desired_initial_configuration << 0, 0, 0, 0, 0, 0, 0;
	desired_initial_configuration << 0,  -45, 0, -115, 0, 60, 60;

	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = GOTO_INITIAL_CONFIG;

	/////////////////////Add some angular movement///////////////
	Vector3d current_position = Vector3d::Zero();
	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task2->_max_velocity = 0.1;

	posori_task2->_kp_pos = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_pos);
	posori_task2->_kp_ori = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_ori);

	VectorXd posori_task2_torques = VectorXd::Zero(dof);
	Vector3d initial_position = Vector3d::Zero();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	//Linear Kalman Filter
    int n = 9; //number of states: position, velocity, acceleration
    int m = 6; // numer of measurements: position, acceleration

    double dt = 1.0/control_freq; //time step

    MatrixXd A = MatrixXd::Zero(n,n); // System dynamics matrix
    MatrixXd C = MatrixXd::Zero(m,n); // Output matrix
    MatrixXd Q = MatrixXd::Zero(n,n); // Process noise covariance
  	MatrixXd R = MatrixXd::Zero(m,m); // Measurement noise covariance
    MatrixXd P = MatrixXd::Identity(n,n); // Estimate error covariance

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
    // Q.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-11, 1.0e-11, 1.0e-11, 1.0e-6, 1.0e-6, 1.0e-6;
    // R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-2, 1.0e-2, 1.0e-3;
    R.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-2, 1.0e-2, 1.0e-2;
    P *= 1;

   //  std::cout << "Q: \n" << Q << std::endl;
   //  std::cout << "R: \n" << R << std::endl;
  	// std::cout << "P: \n" << P << std::endl;

    auto kalman_filter = new KalmanFilter(dt, A, C, Q, R, P);

    //best guess of initial states
    VectorXd x0 = VectorXd::Zero(n);
    //x0 << ;
    double t0 = 0;
    kalman_filter->init(t0, x0);

    double t = 0;
    VectorXd y = VectorXd::Zero(m);

    //position from robot kinematics
    Vector3d position = Vector3d::Zero();
    //velocity from robot kinematics
    Vector3d velocity = Vector3d::Zero();

    //Kalman Filter states
    VectorXd kf_states = VectorXd::Zero(9);
    Vector3d kf_position = Vector3d::Zero();
    Vector3d kf_velocity = Vector3d::Zero();
    Vector3d kf_acceleration = Vector3d::Zero();

    //////////////////////////EKF//////////////////////////
    Quaterniond q_eff = Quaterniond(0,0,0,0);
    VectorXd q_eff_aux = VectorXd::Zero(4);
    //Dimensions
  	int n_ekf = 10; //number of states: quaternion, angular velocity, angular acceleration
  	int m_ekf = 7; //number of measurements: quaternion, angular velocity


  	MatrixXd C_ekf = MatrixXd::Zero(m_ekf,n_ekf); //Output matrix
  	MatrixXd Q_ekf = MatrixXd::Identity(n_ekf,n_ekf); //Process noise covariance
  	MatrixXd R_ekf = MatrixXd::Identity(m_ekf,m_ekf); // Measurement noise covariance
  	MatrixXd P_ekf = MatrixXd::Identity(n_ekf,n_ekf); // Estimate error covariance

  	C_ekf = MatrixXd::Identity(m_ekf, n_ekf);
  	//C_ekf.block(0,0,7,7) = MatrixXd::Identity(7,7);
  	Q_ekf.diagonal() << 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-7, 1.0e-7, 1.0e-7;
  	//R_ekf.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-5, 6e-6, 4.0e-6;
  	//R_ekf.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1, 1, 1;
  	R_ekf.diagonal() << 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1e-1, 1e-1, 1e-1;

  	VectorXd y_ekf = VectorXd::Zero(m_ekf);


  	auto extended_kalman_filter = new QuaternionBasedEKF( dt, C_ekf, Q_ekf, R_ekf, P_ekf);

  	 //best guess of initial states
  	VectorXd x0_ekf = VectorXd::Zero(n_ekf);
  	//x0 << ;
  	extended_kalman_filter->init(t0, x0_ekf);



  	//estimated states
  	VectorXd x_est = VectorXd::Zero(n_ekf);
  	VectorXd q_est = VectorXd::Zero(4);
  	Vector3d avel_est = Vector3d::Zero();
  	Vector3d aaccel_est = Vector3d::Zero();

  	// ///////////////////////////SG Filter///////////////////////////
  	//   // Window size is 2*m+1
	  // const int m_sg = 3;
	  // const int window_sg = 2*m_sg +1;
	  // // Polynomial Order
	  // const int n_sg = 2;
	  // // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
	  // // Points are defined in range [-m;m]
	  // const int t_sg = m_sg;


	  // // Test First Order Derivative
	  //  SavitzkyGolayFilter sg_filter( m_sg, t_sg, n_sg, 1);


	 

	  // VectorXd sg_input = VectorXd::Zero(window_sg);
	  // double sg_output = 0.0;
	  // int sg_counter = 0;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);
		force_moment -= force_torque_bias;



		// cout << "FT calibrated: " << force_moment.transpose() << endl; 
		
		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);

			// get object position in Base coordinates
			robot->position(position, link_name, Eigen::Vector3d::Zero());

			// get object acceleration and velocity in Base coordinates
			robot->linearAcceleration(accel,link_name, Eigen::Vector3d::Zero());
			robot->linearVelocity(velocity, link_name, Eigen::Vector3d::Zero());
			robot->angularVelocity(avel,link_name);
			robot->angularAcceleration(aaccel, link_name);
			//compute Transformation base to sensor frame in base coordinates
			Matrix3d R_link = Matrix3d::Zero();
			robot->rotation(R_link,link_name); 
			//get object linear acc in sensor frame
			accel_local = R_link.transpose()*accel;
			//get object angular velocity in sensor frame
			avel_local = R_link.transpose()*avel;
			// get object angular acceleration in sensor frame
			aaccel_local = R_link.transpose()*aaccel;
			//get object linear velocity in sensor frame
			velocity = R_link.transpose()*velocity;	
			//get object position in sensor frame
			position = R_link.transpose()*position;		
			g_local = R_link.transpose()*robot->_world_gravity;

			//remove local gravity from acceleration
			accel_local += g_local;

			t+= dt;
            y << position, accel_local;
            kalman_filter->update(y);
            //std::cout << "t = " << t << ", \n" << "y= " << y.transpose()
            //<< ",\n x_hat= " << kalman_filter->state().transpose() << std::endl;

            /////////////////////EKF/////////////////////////
            q_eff = R_link.transpose();
			q_eff_aux << q_eff.w(), q_eff.vec();
			y_ekf << q_eff_aux, avel_local;
			extended_kalman_filter-> update(y_ekf);
			x_est = extended_kalman_filter->state();
 			q_est << x_est(0), x_est(1), x_est(2), x_est(3);
 			//std::cout << "q_est" << q_est.transpose() << std::endl;
 			avel_est << x_est(4), x_est(5), x_est(6);
 			//std::cout << "avel_est" << avel_est.transpose() << std::endl;
 			aaccel_est << x_est(7), x_est(8), x_est(9);
 			//std::cout << "aaccel_est" << aaccel_est << std::endl;
			
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

			//To compare KF Data
			robot->position(position, link_name, Eigen::Vector3d::Zero());
			robot->linearVelocity(velocity, link_name, Eigen::Vector3d::Zero());
			//compute Transformation base to sensor frame in base coordinates
			Matrix3d R_link = Matrix3d::Zero();
			robot->rotation(R_link,link_name); 
			//get object linear velocity in sensor frame
			velocity = R_link.transpose()*velocity;	
			//get object position in sensor frame
			position = R_link.transpose()*position;	

			//////////////////////To compare sensor data/////////////////////////
			robot->linearAcceleration(accel_kinematics,link_name, Eigen::Vector3d::Zero());
			robot->angularVelocity(avel_kinematics,link_name);
			robot->angularAcceleration(aaccel_kinematics, link_name);
			g_local = R_link.transpose()*robot->_world_gravity;
			accel_kinematics = R_link.transpose()*accel_kinematics;
			accel_kinematics +=g_local;
			avel_kinematics = R_link.transpose()*avel_kinematics;
			aaccel_kinematics = R_link.transpose()*aaccel_kinematics;
			/////////////////////////////////////////////////////////////

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
			if(controller_counter%2 ==0) //sample data at 500 Hz
			{
				// std::cout << "logging" << std::endl;
				accel = redis_client.getEigenMatrixJSON(ACCELEROMETER_DATA_KEY);
				// std::cout <<"accelerometer data: " << accel.transpose() << std::endl;
				avel = redis_client.getEigenMatrixJSON(GYROSCOPE_DATA_KEY);
				// std::cout <<"gyroscope data: " << avel.transpose() << std::endl;
				accel_gravity_removed = redis_client.getEigenMatrixJSON(LINEAR_ACCELERATION_CORRECTED_KEY);
			}
			// 
			// accel_gravity_removed = R_acc_in_ft*accel_gravity_removed;
			accel_gravity_removed *= 9.81;
			accel_local = R_acc_in_ft*accel;
			accel_local *= 9.81; //Accelerometer output is in g
			accel_local += g_local;
			//accel_local = filter->update(aaccel_local);
			// avel_local_unfiltered = R_acc_in_ft*avel;
			avel_local = R_acc_in_ft*avel;
			avel_local *=  (2*M_PI)/360; //Gyroscope output is in deg/s
			t+=dt;
			y << position, accel_local;
			kalman_filter->update(y);
			//avel_local_last = avel_local;

			/////////////////EKF////////////////
			q_eff = R_link.transpose();
			q_eff_aux << q_eff.w(), q_eff.vec();
			y_ekf << q_eff_aux, avel_local;
			extended_kalman_filter-> update(y_ekf);
			x_est = extended_kalman_filter->state();

			Eigen::VectorXd f = Eigen::VectorXd::Zero(10);
            f = extended_kalman_filter->nonlinear_sys_dyn();
            std::cout << "nonlinear_sys_dyn" << f.transpose() << std::endl;

            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(10,10);
            F = extended_kalman_filter->linearized_sys_dyn();
            std::cout << "linearized sys dyn" << F << std::endl;

            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(10,10);
            P = extended_kalman_filter->error_cov();
            std::cout << "error cov" << P << std::endl;

 			q_est << x_est(0), x_est(1), x_est(2), x_est(3);
 			std::cout << "q_est" << q_est.transpose() << std::endl;
 			avel_est << x_est(4), x_est(5), x_est(6);
 			std::cout << "avel_est" << avel_est.transpose() << std::endl;
 			aaccel_est << x_est(7), x_est(8), x_est(9);
 			std::cout << "aaccel_est" << aaccel_est << std::endl;

 			

 			//////////////////SG////////////////
 			// if(sg_counter < window_sg)
 			// {
 			// 	sg_input(sg_counter) = avel_local[0];
 			// 	sg_counter++;
 			// 	std::cout << "input" << sg_input << std::endl;
 			// }
 			// else if(sg_counter == window_sg)
 			// {
 			// 	sg_output = sg_filter.filter(sg_input, 0.);
 			// 	std::cout << "output" << sg_output << std::endl;
 			// 	sg_counter = 0;
 			// }

		}
		kf_states = kalman_filter->state();
		kf_position << kf_states(0), kf_states(1), kf_states(2);
		kf_velocity << kf_states(3), kf_states(4), kf_states(5);
		kf_acceleration << kf_states(6), kf_states(7), kf_states(8);

		// cout << "full: \n" << kalman_filter->state() << endl;
		// cout << "pos: \n" << kf_position << endl;
		// cout << "vel: \n" << kf_velocity << endl;
		// cout << "acc: \n" << kf_acceleration << endl; 

		// state machine
		if(state == GOTO_INITIAL_CONFIG)
		{	
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;
			
			


			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
			if(config_error.norm() < 0.26)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));


				state = MOVE_TO_OBJECT;
				
			}

		}
		if(state == MOVE_TO_OBJECT)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			posori_task2->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			Vector3d desired_position = Vector3d(.4,0.3,0.4);

			posori_task->_goal_position = desired_position;

			robot->position(current_position, posori_task->_link_name, pos_in_link);
			double circle_radius = 0.15;
			double circle_freq = 0.5;
			posori_task2->_desired_position = circle_radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()));
			posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, 0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()));

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			posori_task2->computeTorques(posori_task2_torques);
			joint_task->computeTorques(joint_task_torques);
			
			command_torques = posori_task_torques + joint_task_torques + coriolis;
			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;

			if(config_error_posori.norm() < 0.2)
			{	
				counter++; 
				if(counter==1500)
				{
					joint_task->reInitializeTask();
					posori_task->reInitializeTask();
					posori_task->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
					posori_task2->reInitializeTask();
					posori_task2->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
					//Vector3d desired_position = Vector3d(.4,-0.3,0.4);
					//posori_task->_goal_position = desired_position;
					state = MOVE;	
				}
			}




		}
		if(state == MOVE)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			posori_task2->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,-0.5,0.4);

			robot->position(current_position, posori_task->_link_name, pos_in_link);
			double circle_radius = 0.3;
			double circle_freq = 0.3;
			// posori_task2->_desired_position = current_position + circle_radius * Eigen::Vector3d(sin(2*M_PI*circle_freq*timer.elapsedTime()), cos(2*M_PI*circle_freq*timer.elapsedTime()), 0.0);
			// posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(cos(2*M_PI*circle_freq*timer.elapsedTime()), -sin(2*M_PI*circle_freq*timer.elapsedTime()),0.0);
			posori_task2->_desired_position = current_position + circle_radius * Eigen::Vector3d(0.0, 0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()));
			posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, 0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()));

			// compute torques

			posori_task2->computeTorques(posori_task2_torques);
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis + posori_task2_torques;


			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
				joint_task->_goal_position(5) += M_PI/2;
				state = HAND_OVER;
				
			}

		}

		// if(state == MOVE2)
		// {
		// 	// update tasks models
		// 	N_prec.setIdentity();
		// 	posori_task->updateTaskModel(N_prec);
		// 	posori_task2->updateTaskModel(N_prec);
		// 	N_prec = posori_task->_N;
		// 	joint_task->updateTaskModel(N_prec);

			
		// 	robot->position(current_position, posori_task->_link_name, pos_in_link);
		// 	double circle_radius = 0.3;
		// 	double circle_freq = 0.2;
		// 	posori_task2->_desired_position = current_position + circle_radius * Eigen::Vector3d(0.0, sin(2*M_PI*circle_freq*timer.elapsedTime()), 0.0);
		// 	posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(0.0, cos(2*M_PI*circle_freq*timer.elapsedTime()), 0.0);

		// 	// compute torques

		// 	posori_task2->computeTorques(posori_task2_torques);

		// 	posori_task->computeTorques(posori_task_torques);

		// 	joint_task->computeTorques(joint_task_torques);

		// 	command_torques = posori_task_torques + joint_task_torques +  posori_task2_torques + coriolis;


		// 	VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
		// 	if(config_error_posori.norm() < 0.2)
		// 	{
		// 		joint_task->reInitializeTask();
		// 		posori_task->reInitializeTask();
		// 		posori_task->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
		// 		posori_task2->reInitializeTask();
		// 		posori_task2->enableVelocitySaturation(Eigen::Vector3d(0.2,0.2,0.2), Eigen::Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0));
		// 		joint_task->_goal_position(5) += M_PI/2;
		// 		state = HAND_OVER;
				
		// 	}

		//  }


		if(state == HAND_OVER)
		{
			// joint_task->_kp = 40;
			// joint_task->_kv = 2.1 * sqrt(joint_task->_kp);
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			
			N_prec = joint_task->_N_prec;
			posori_task->updateTaskModel(N_prec);


			


			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}



		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_LOCAL_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_ACCELERATION_LOCAL_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(KALMAN_FILTER_POS_KEY, kf_position);
		redis_client.setEigenMatrixDerived(KALMAN_FILTER_VEL_KEY, kf_velocity);
		redis_client.setEigenMatrixDerived(KALMAN_FILTER_ACC_KEY, kf_acceleration);
		redis_client.setEigenMatrixDerived(POSITION_KEY, position);
		redis_client.setEigenMatrixDerived(LINEAR_VELOCITY_KEY, velocity);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local); 
		redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_CORRECTED_RIGHT_FRAME_KEY, accel_gravity_removed);

		//DELETE AFTER CHECKING
		redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_KINEMATICS_KEY, accel_kinematics);
		redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_KINEMATICS_KEY, avel_kinematics);
		redis_client.setEigenMatrixDerived(ANGULAR_ACCELERATION_KINEMATICS_KEY, aaccel_kinematics);

		//test the EKF
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_ESTIMATE_KEY, avel_est);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_ESTIMATE_KEY, aaccel_est);
		redis_client.setEigenMatrixDerived(QUATERNION_KEY, q_eff_aux);
		redis_client.setEigenMatrixDerived(QUATERNION_ESTIMATE_KEY, q_est);




	

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