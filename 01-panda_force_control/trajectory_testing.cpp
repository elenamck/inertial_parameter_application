#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <tinyxml2.h>
#include <chrono>



#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";


#define  GOTO_INITIAL_CONFIG  0
#define  MOVE_TO_OBJECT        		 1
#define  MOVE 						 2
#define  HAND_OVER 						 3
#define  GOTO_CONTACT         4
#define  INTO_CONTACT         5
#define MOVE_UP 6
#define MOVE_DOWN 7

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

const bool inertia_regularization = true;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string KALMAN_FILTER_POS_KEY;
std::string KALMAN_FILTER_VEL_KEY;
std::string KALMAN_FILTER_ACC_KEY;
std::string ANGULAR_VEL_ESTIMATE_KEY;
std::string ANGULAR_ACC_ESTIMATE_KEY;
std::string QUATERNION_KEY;
std::string QUATERNION_ESTIMATE_KEY;
std::string ANGULAR_VEL_KEY;
std::string ANGULAR_ACC_KEY;
std::string LINEAR_ACC_KEY;
std::string LOCAL_GRAVITY_KEY;
std::string INERTIAL_PARAMS_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;
std::string DESIRED_POS_KEY;
std::string CURRENT_POS_KEY;
std::string FORCE_VIRTUAL_KEY;
std::string ACCELEROMETER_DATA_KEY;
std::string GYROSCOPE_DATA_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;


/* Data Matrix Force/Torque virtual A_t based on 
 *"Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
 */
Eigen::MatrixXd GetDataMatrixFT(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local);
/* Data Matrix Force/Torque virtual A_t based on 
 * "Improving Force Control Performance by Computational Elimination of Non-Contact Forces/Torques", D. Kubus, T. Kroeger, F. Wahl, ICRA 2008  
*/
Eigen::MatrixXd computeSigma(Eigen::MatrixXd K, Eigen::MatrixXd Sigma, Eigen::MatrixXd A);
Eigen::MatrixXd computeK(Eigen::MatrixXd Sigma, Eigen::MatrixXd A, Eigen::MatrixXd Lambda);

void computeInertialNew(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma); 

int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::DemoApplication::force_sesnor::force_moment";
		DESIRED_POS_KEY = "sai2::DemoApplication::Panda::controller::logging::desired_position";
		CURRENT_POS_KEY = "sai2::DemoApplication::Panda::controller::logging::current_position";
		FORCE_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";
		LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";
		LOCAL_GRAVITY_KEY = "sai2::DemoApplication::Panda::simulation::g_local";
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";


	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";	

		//Keys for inertial parameter estimation
		INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::inertial_parameter";
		ANGULAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::avel";
		ANGULAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::aaccel";
		LINEAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::accel";

		//Keys for Kalman Filter
		KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::position";
		KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
		KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::acceleration";

		//Keys for Extended Kalman Filter
		ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_vel";
		ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_acc";
		QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::logging::quaternion";
		QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::quaternion";

		//Sensor Keys
		ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";        // in local frame
		GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";

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
	VectorXd force_moment = Eigen::VectorXd::Zero(6);
	force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_KEY);
	//-----FT sensor (simfile: panda_arm_FT_sensor.urdf)------
	VectorXd force_virtual = Eigen::VectorXd::Zero(6);
	//-----F sensor (simfile: panda_arm_force_sensor.urdf)------
	// VectorXd force_virtual = Eigen::VectorXd::Zero(3);


	Vector3d current_position = Eigen::Vector3d::Zero();
	Vector3d desired_position = Eigen::Vector3d::Zero();


	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);
	MatrixXd Jv = MatrixXd::Identity(3, dof);


	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.1);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.11;

	posori_task->_kp_pos = 60.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_pos);
	posori_task->_kp_ori = 50.0;
	posori_task->_kv_pos = 2.1*sqrt(posori_task->_kp_ori);

	Vector3d vel_sat = Vector3d(0.2,0.2,0.2);
	Vector3d avel_sat = Vector3d(M_PI/3.0, M_PI/3.0, M_PI/3.0);
	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = vel_sat;
	posori_task->_angular_saturation_velocity = avel_sat;


	// position controller for parameter estimation

	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task2->_max_velocity = 0.1;

	posori_task2->_kp_pos = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_pos);
	posori_task2->_kp_ori = 35.0;
	posori_task2->_kv_pos = 2.1*sqrt(posori_task2->_kp_ori);
	posori_task2->_velocity_saturation = true;
	posori_task2->_linear_saturation_velocity = vel_sat;
	posori_task2->_angular_saturation_velocity = avel_sat;
	VectorXd posori_task2_torques = VectorXd::Zero(dof);


	//For inertial parameter estimation

	Eigen::Vector3d accel = Eigen::Vector3d::Zero(); //object linear acceleration in base frame
	Eigen::Vector3d avel = Eigen::Vector3d::Zero(); //object angular velocity in base frame
	Eigen::Vector3d aaccel = Eigen::Vector3d::Zero(); //object angular acceleration in base frame
	Eigen::Vector3d accel_local = Eigen::Vector3d::Zero(); // object linear acceleration in sensor frame
	Eigen::Vector3d aaccel_local = Eigen::Vector3d::Zero(); // object angular acceleration in sensor frame
	Eigen::Vector3d avel_local = Eigen::Vector3d::Zero(); //object angular velocity in sensor frame
	Eigen::Vector3d g_local = Eigen::Vector3d::Zero(); //gravity vector in sensor frame
	Eigen::MatrixXd A_data = Eigen::MatrixXd::Zero(6,10); //Data matrix
	Eigen::VectorXd phi = Eigen::VectorXd::Zero(10); //inertial parameter vector
	Eigen::VectorXd FT = Eigen::VectorXd::Zero(6); //FT Measurements
	Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(10,10);
	Eigen::MatrixXd Inertia = Eigen::MatrixXd::Zero(3,3);
	Eigen::Vector3d com = Eigen::Vector3d::Zero(3);
	int n = 0;
	int n_measurements = 0;
	Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
	Eigen::Matrix3d initial_orientation = Eigen::Matrix3d::Zero();
	robot->rotation(initial_orientation, posori_task->_link_name);
	robot->position(initial_position, posori_task->_link_name, pos_in_link);

	VectorXd posori_task_torques = VectorXd::Zero(dof);




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
	int counter =0;

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
		force_moment = redis_client.getEigenMatrixJSON(FORCE_VIRTUAL_KEY);

		A_data = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);
		computeInertialNew(n_measurements, force_moment, A_data, phi,  Sigma);
		n_measurements++;

		robot->linearAcceleration(accel,link_name, Eigen::Vector3d::Zero());
		robot->angularAcceleration(aaccel,link_name);
		robot->angularVelocity(avel,link_name);
		//compute Transformation base to sensor frame in base coordinates
		Eigen::Matrix3d R_link;
		robot->rotation(R_link,link_name); 
		//get object acc in sensor frame
		accel_local = R_link.transpose()*accel;
		aaccel_local = R_link.transpose()*aaccel;
		//get object velocity in sensor frame
		avel_local = R_link.transpose()*avel;
		// get gravity in base frame and transform to sensor frame
		g_local = R_link.transpose()*robot->_world_gravity;

		accel_local += g_local; 




std::chrono::high_resolution_clock::time_point t_start;
std::chrono::duration<double> t_elapsed;


t_start = std::chrono::high_resolution_clock::now();





		Vector3d sensed_force_sensor_frame = force_moment.head(3);
		Vector3d sensed_moment_sensor_frame = force_moment.tail(3);


t_elapsed =  std::chrono::high_resolution_clock::now() - t_start;



		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
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
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);


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
					posori_task->enableVelocitySaturation(vel_sat, avel_sat);
					posori_task2->reInitializeTask();
					posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
					state = MOVE_UP;	
				}
			}




		}

		if(state == MOVE_UP)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,0.3,0.9);



			// compute torques

			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;



			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE_DOWN;
				
			}

		}


		if(state == MOVE_DOWN)
		{
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_goal_position = Vector3d(.4,0.3,0.4);



			// compute torques

			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;



			VectorXd config_error_posori = posori_task->_goal_position - posori_task->_current_position;
			if(config_error_posori.norm() < 0.2)
			{
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				state = MOVE;
				
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
				posori_task->enableVelocitySaturation(vel_sat, avel_sat);
				posori_task2->reInitializeTask();
				posori_task2->enableVelocitySaturation(vel_sat, avel_sat);
				joint_task->_goal_position(5) += M_PI/2;
				state = HAND_OVER;
				
			}

		}


		if(state == HAND_OVER)
		{

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
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi);


		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    Inertia << phi(4), phi(5), phi(6), phi(5), phi(7), phi(8), phi(6), phi(8), phi(9); 
	com << phi(1)/phi(0), phi(2)/phi(0), phi(3)/phi(0); 
    std::cout << "estimated mass" << phi(0) << "\n";
    std::cout << "estimated center of mass" << 	com.transpose() << "\n";
    std::cout << "estimated Inertia" << Inertia << "\n";


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


void computeInertialNew(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma)
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
