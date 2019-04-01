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

const string robot_file = "../../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

#define  PARAMETER_ESTIMATION 0
#define  GOTO_INITIAL_CONFIG  1
#define  GOTO_CONTACT         2
#define  INTO_CONTACT         3

unsigned long long controller_counter = 0;

const bool flag_simulation = true;
// const bool flag_simulation = false;

const bool inertia_regularization = true;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string FORCE_COMPUTED_KEY;
std::string INERTIAL_PARAMS_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;
std::string DESIRED_POS_KEY;
std::string CURRENT_POS_KEY;
std::string FORCE_VIRTUAL_KEY;
std::string ANGULAR_VEL_KEY;
std::string ANGULAR_ACC_KEY;
std::string LINEAR_ACC_KEY;
std::string LOCAL_GRAVITY_KEY;


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
Eigen::MatrixXd GetDataMatrixLin(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local);

/* Adding Measurements Force/Torque to the Data Matrix
 */
void addMeasurementFT(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A, Eigen::VectorXd& FT, Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, const int filter_size);
/* Adding Measurements Force/Torque to the Data Matrix
 */
void addMeasurementLin(int& n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A_lin, Eigen::VectorXd& F, Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local);

/* function to calculate inertial parameter vector (10) based on least squares
 * consisting of:
 * mass, coordinates of center of mass, elements of the inertia matrix
 */
Eigen::VectorXd getInertialParameterVectorFull(Eigen::VectorXd& phi, Eigen::MatrixXd A, Eigen::VectorXd FT);
/* function to calculate inertial parameter vector (4) based on least squares 
 * consisting of mass, coordinates of center of mass
 */
Eigen::VectorXd getInertialParameterVectorLin(Eigen::VectorXd& phi, Eigen::MatrixXd A_lin, Eigen::VectorXd F);

/* function to compute the contact force
 * f_contact = f_sensor - f_inertia
 */
Eigen::VectorXd computeContactForce(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, Eigen::VectorXd force_virtual, Eigen::VectorXd phi);
Eigen::MatrixXd computeSigma(Eigen::MatrixXd K, Eigen::MatrixXd Sigma, Eigen::MatrixXd A);
Eigen::MatrixXd computeK(int n_measurements, Eigen::MatrixXd Sigma, Eigen::MatrixXd A, Eigen::MatrixXd Lambda);


void computeInertial(int& n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A, Eigen::VectorXd& FT, Eigen::Vector3d accel_local,
					 Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma, const int filter_size);

void computeInertialNew(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd A, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma); 

int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
		DESIRED_POS_KEY = "sai2::DemoApplication::Panda::controller::logging::desired_position";
		CURRENT_POS_KEY = "sai2::DemoApplication::Panda::controller::logging::current_position";
		FORCE_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
		FORCE_COMPUTED_KEY = "sai2::DemoApplication::Panda::simulation::computed_force";
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
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.15);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);



	posori_task->_max_velocity = 0.1;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_ki_pos = 2.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;
	posori_task->_ki_ori = 2.0;


	// position controller for parameter estimation

	auto posori_task2 = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task2->_max_velocity = 0.1;
	posori_task2->_kp_pos = 100.0;
	posori_task2->_kv_pos = 20.0;
	posori_task2->_ki_pos = 2.0;
	posori_task2->_kp_ori = 100.0;
	posori_task2->_kv_ori = 20.0;
	posori_task2->_ki_ori = 2.0;

	posori_task2->_velocity_saturation = true;
	posori_task2->_linear_saturation_velocity = Eigen::Vector3d(0.5,0.5,0.5);
	posori_task2->_angular_saturation_velocity = Eigen::Vector3d(M_PI/4.0, M_PI/4.0, M_PI/4.0);
	Eigen::VectorXd posori_task2_torques = Eigen::VectorXd::Zero(dof);


	//For inertial parameter estimation
	Eigen::VectorXd obj_accel = Eigen::VectorXd::Zero(6); //object acceleration in sensor frame
	Eigen::Vector3d accel = Eigen::Vector3d::Zero(); //object linear acceleration in base frame
	Eigen::Vector3d avel = Eigen::Vector3d::Zero(); //object angular velocity in base frame
	Eigen::Vector3d aaccel = Eigen::Vector3d::Zero(); //object angular acceleration in base frame
	Eigen::Vector3d accel_local = Eigen::Vector3d::Zero(); // object linear acceleration in sensor frame
	Eigen::Vector3d aaccel_local = Eigen::Vector3d::Zero(); // object angular acceleration in sensor frame
	Eigen::Vector3d avel_local = Eigen::Vector3d::Zero(); //object angular velocity in sensor frame
	Eigen::Vector3d g_local = Eigen::Vector3d::Zero(); //gravity vector in sensor frame
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,10); //Data matrix
	Eigen::MatrixXd A_lin = Eigen::MatrixXd::Zero(3,4);
	Eigen::VectorXd phi = Eigen::VectorXd::Zero(10); //inertial parameter vector
	Eigen::VectorXd FT = Eigen::VectorXd::Zero(6); //FT Measurements
	Eigen::VectorXd F = Eigen::VectorXd::Zero(3); //F Measurements
	Eigen::VectorXd f_contact = Eigen::VectorXd::Zero(6); //computed contact force
	Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(10,10);
	Eigen::MatrixXd Inertia = Eigen::MatrixXd::Zero(3,3);
	Eigen::Vector3d com = Eigen::Vector3d::Zero(3);
	int n = 0;
	int n_measurements = 0;

	Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
	Eigen::Matrix3d initial_orientation = Eigen::Matrix3d::Zero();
	Eigen::Affine3d T;
	robot->rotation(initial_orientation, posori_task2->_link_name);
	cout << "initial_orientation: \n" << initial_orientation << endl;
	robot->position(initial_position, posori_task2->_link_name, pos_in_link);
	cout << "initial_position: \n" << initial_position.transpose() << endl;



	VectorXd posori_task_torques = VectorXd::Zero(dof);

	//set sensor frame
	Affine3d transformation_in_link = Eigen::Affine3d::Identity(); 
	transformation_in_link.translation() = pos_in_link;
	posori_task->setForceSensorFrame(link_name, transformation_in_link);



	// joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = 0.2;

	joint_task->_kp = 10.0;
	joint_task->_kv = 5.0;

	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	desired_initial_configuration << 0, 10, 0, -125, 0, 135, 0;
	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = PARAMETER_ESTIMATION;

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
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_KEY);
		force_virtual = redis_client.getEigenMatrixJSON(FORCE_VIRTUAL_KEY);
		// accel_local = redis_client.getEigenMatrixJSON(LINEAR_ACC_KEY);
		// aaccel_local = redis_client.getEigenMatrixJSON(ANGULAR_ACC_KEY);
		// avel_local = redis_client.getEigenMatrixJSON(ANGULAR_VEL_KEY);
		// g_local = redis_client.getEigenMatrixJSON(LOCAL_GRAVITY_KEY);
		// get object acceleration and velocity in Base coordinates
		robot->linearAcceleration(accel,link_name, Eigen::Vector3d::Zero());
		robot->angularAcceleration(aaccel,link_name);
		robot->angularVelocity(avel,link_name);
		//compute Transformation base to sensor frame in base coordinates
		Eigen::Matrix3d R_link;
		robot->rotation(R_link,link_name); 
		//get object acc in sensor frame
		accel_local = R_link.transpose()*accel;
		aaccel_local = R_link.transpose()*aaccel;
		obj_accel << accel_local, aaccel_local;
		//get object velocity in sensor frame
		avel_local = R_link.transpose()*avel;
		// get gravity in base frame and transform to sensor frame
		g_local = R_link.transpose()*robot->_world_gravity;




std::chrono::high_resolution_clock::time_point t_start;
std::chrono::duration<double> t_elapsed;


t_start = std::chrono::high_resolution_clock::now();

/* some code */



		Vector3d sensed_force_sensor_frame = force_moment.head(3);
		Vector3d sensed_moment_sensor_frame = force_moment.tail(3);

		//std::cout << "Force sensed:  " << force_moment.transpose() << "\n";
   		//std::cout << "Force virtual: " << force_virtual.transpose() << "\n";
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


		// state machine

		if(state == PARAMETER_ESTIMATION)
		{
		robot->position(current_position, link_name, pos_in_link);

		N_prec.setIdentity();
		posori_task2->updateTaskModel(N_prec);


		double circle_radius = 0.00002*n;
		double circle_freq = 0.25;
		double time = controller_counter/control_freq;



		if(controller_counter<8000)
		{
			 Eigen::Matrix3d R;
			 double theta = M_PI/3.0;
			 R << cos(theta) , 0 , sin(theta),
			 	      0     , 1 ,     0     ,
			     -sin(theta) , 0 , cos(theta);
			posori_task2->_desired_position = initial_position + circle_radius * Eigen::Vector3d(0.0, sin(2*M_PI*circle_freq*time), cos(2*M_PI*circle_freq*time));

			posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(0.0, cos(2*M_PI*circle_freq*time), -sin(2*M_PI*circle_freq*time));
			
			posori_task2->_desired_orientation = R.transpose()*initial_orientation;

			n++;
		}
		else if(controller_counter < 16000)

			{	
				if (controller_counter ==8000)
				{
					n=0;
				}
				
				Eigen::Matrix3d R;
				 double theta = M_PI/3.0;
				 R << cos(theta) , -sin(theta),  0,
				      sin(theta) ,  cos(theta),  0,
				           0     ,       0    ,  1;
				
				posori_task2->_desired_position = initial_position + circle_radius * Eigen::Vector3d( sin(2*M_PI*circle_freq*time), 0.0,cos(2*M_PI*circle_freq*time));
				posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(cos(2*M_PI*circle_freq*time),0.0,  -sin(2*M_PI*circle_freq*time));


				posori_task2->_desired_orientation = R.transpose()*initial_orientation;
				n++;

			}

		else if(controller_counter < 30000)

			{	
				if (controller_counter ==16000)
				{
					n=0;
				}
				
				Eigen::Matrix3d R;
				double theta = M_PI/2.0;
				R << 1 , 	0, 			 0,
				   	 0 , cos(theta), -sin(theta),
				     0 , sin(theta),   cos(theta) ;
				
				posori_task2->_desired_position = initial_position + circle_radius * Eigen::Vector3d( sin(2*M_PI*circle_freq*time), cos(2*M_PI*circle_freq*time), 0.0);
				posori_task2->_desired_velocity = 2*M_PI*circle_freq*circle_radius*Eigen::Vector3d(cos(2*M_PI*circle_freq*time),  -sin(2*M_PI*circle_freq*time), 0.0);
				posori_task2->_desired_orientation = R.transpose()*initial_orientation;
				n++;

			}


		// compute task torques

		posori_task2->computeTorques(posori_task2_torques);

		command_torques = posori_task2_torques + coriolis;


		A = GetDataMatrixFT(accel_local, avel_local, aaccel_local,  g_local);

		computeInertialNew(n_measurements, force_virtual, A, phi,  Sigma);

 		n_measurements++;


		if(controller_counter ==30000)
			{
				posori_task2->reInitializeTask();
				posori_task->_goal_position(2) -= 0.2;


				state = GOTO_INITIAL_CONFIG;
			}

 	 	}


    			// state machine
		if(state == GOTO_INITIAL_CONFIG)
		{	

			robot->updateModel();
			robot->position(current_position, link_name, pos_in_link);
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;


			if(config_error.norm() < 0.05)
			{
				posori_task->reInitializeTask();
				posori_task->_goal_position(2) -= 0.2;


				state = GOTO_CONTACT;
			}

		}
		else if(state == GOTO_CONTACT)
		{	

			robot->updateModel();
			robot->position(current_position, link_name, pos_in_link);
			desired_position = posori_task->_desired_position;
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);


			command_torques = posori_task_torques + joint_task_torques + coriolis;

				// eff position
			current_position = Jv*robot->_q;
			std::cout << "sensed force:  " << force_moment.transpose() << "\n";

			if(force_moment.norm() > 0.0005)
			{
				std::cout << "sensed force contact transition:  " << force_moment.transpose() << "\n";
				posori_task->reInitializeTask();
				// write initial position and orientation
				robot->rotation(initial_orientation, posori_task->_link_name);
				robot->position(initial_position, posori_task->_link_name, pos_in_link);

				state = INTO_CONTACT; 
			}
		}

		else if(state == INTO_CONTACT)
		{
			robot->updateModel();
			robot->position(current_position, link_name, pos_in_link);
			desired_position = posori_task->_desired_position;
			// update tasks models
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);


			//desired position component
			posori_task->_desired_angular_velocity = Eigen::Vector3d::Zero();
			double circle_radius = 0.1;
			double circle_freq = 0.25;
			posori_task->_desired_position = initial_position + circle_radius * Eigen::Vector3d(sin(2*M_PI*circle_freq*timer.elapsedTime()), cos(2*M_PI*circle_freq*timer.elapsedTime()), 0.0);
			posori_task->_desired_velocity = 2*M_PI*circle_freq*circle_radius * Eigen::Vector3d(cos(2*M_PI*circle_freq*timer.elapsedTime()), -sin(2*M_PI*circle_freq*timer.elapsedTime()),0.0);



			//update sensed forces and moments
			posori_task->updateSensedForceAndMoment(sensed_force_sensor_frame, sensed_moment_sensor_frame);

			//set force axis
			Vector3d force_axis = Vector3d::Zero(3);
			force_axis << 0,0,1;
			posori_task->setForceAxis(force_axis);


			//desired force component
			posori_task->_desired_force << 0,0,-10;
			posori_task->_desired_moment << 0,0,0;


			// compute torques
			posori_task->computeTorques(posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			f_contact = computeContactForce(accel_local, avel_local, aaccel_local,g_local, force_virtual, phi);

			std::cout << "Force sensed:  " << force_moment.transpose() << "\n";
   			std::cout << "Force virtual: " << force_virtual.transpose() << "\n";
   			std::cout << "Force contact: " << f_contact.transpose() << "\n";


		}

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(CURRENT_POS_KEY, current_position);
		redis_client.setEigenMatrixDerived(DESIRED_POS_KEY, desired_position);
		redis_client.setEigenMatrixDerived(FORCE_COMPUTED_KEY, f_contact);
		redis_client.setEigenMatrixDerived(ANGULAR_ACC_KEY, aaccel_local);
		redis_client.setEigenMatrixDerived(LINEAR_ACC_KEY, accel_local);
		redis_client.setEigenMatrixDerived(ANGULAR_VEL_KEY, avel_local);
		redis_client.setEigenMatrixDerived(LOCAL_GRAVITY_KEY, g_local);
		redis_client.setEigenMatrixDerived(INERTIAL_PARAMS_KEY, phi);

		if(controller_counter%1000==0)
		{
    		Inertia << phi(4), phi(5), phi(6), phi(5), phi(7), phi(8), phi(6), phi(8), phi(9); 
			com << phi(1)/phi(0), phi(2)/phi(0), phi(3)/phi(0); 
			cout << "the estimated mass is: \n" << phi(0) << endl;
		    cout << "the estimated center of mass is: \n" << com.transpose() << endl;
		    cout << "the estimated inertia tensor is: \n" << Inertia << endl;
		}

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


Eigen::MatrixXd GetDataMatrixLin(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local)
{
	Eigen::MatrixXd A_full = GetDataMatrixFT(accel_local, avel_local,  aaccel_local, g_local);
	Eigen::MatrixXd A_lin = Eigen::MatrixXd::Zero(3,4);
	A_lin = A_full.block(0,0,3,4);

	return A_lin;
}

void addMeasurementFT(int n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A, Eigen::VectorXd& FT, Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, const int filter_size)
{	
	Eigen::MatrixXd a = GetDataMatrixFT(accel_local, avel_local,  aaccel_local, g_local);
	Eigen::VectorXd ft = force_virtual;

	if(n_measurements == 1)
		{
			A = a;
			FT = ft;
		}

	else if (n_measurements<filter_size+1)
		{
			Eigen::MatrixXd A_temp = A;
			Eigen::VectorXd FT_temp = FT;

			A.resize(n_measurements*6, 10);
			FT.resize(n_measurements*6);

			A.topRows((n_measurements-1)*6) = A_temp;
			FT.topRows((n_measurements-1)*6) = FT_temp;

			A.bottomRows(6) = a;
			FT.bottomRows(6) = ft;
		}

	else if (n_measurements==filter_size+1)
	{
				std::cout << "error in adding measurement" << n_measurements << "\n";

	}

}

void addMeasurementLin(int& n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A_lin, Eigen::VectorXd& F, Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local)
{	

	Eigen::MatrixXd a = GetDataMatrixLin(accel_local, avel_local,  aaccel_local, g_local);
	Eigen::VectorXd f = force_virtual;

	if(n_measurements == 1)
		{
			A_lin = a;
			F = f;
		}

	else
		{
			Eigen::MatrixXd A_temp = A_lin;
			Eigen::VectorXd F_temp = F;

			A_lin.resize(n_measurements*3, 4);
			F.resize(n_measurements*3);

			A_lin.topRows((n_measurements-1)*3) = A_temp;
			F.topRows((n_measurements-1)*3) = F_temp;

			A_lin.bottomRows(3) = a;
			F.bottomRows(3) = f;
		}

}

Eigen::VectorXd getInertialParameterVectorFull(Eigen::VectorXd& phi, Eigen::MatrixXd A, Eigen::VectorXd FT)
{
	phi = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(FT);

	return phi;
}

Eigen::VectorXd getInertialParameterVectorLin(Eigen::VectorXd& phi, Eigen::MatrixXd A_lin, Eigen::VectorXd F)
{
	phi = A_lin.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(F);
	phi(1) = phi(1)/phi(0);
	phi(2) = phi(1)/phi(0);
	phi(3) = phi(1)/phi(0);

	return phi;
}

Eigen::VectorXd computeContactForce(Eigen::Vector3d accel_local, Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, Eigen::VectorXd force_virtual , Eigen::VectorXd phi)
{	

	Eigen::MatrixXd A_data = GetDataMatrixFT( accel_local, avel_local,  aaccel_local,  g_local);
	Eigen::VectorXd f_contact = force_virtual - A_data * phi;

	return f_contact;
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

void computeInertial(int& n_measurements, Eigen::VectorXd force_virtual, Eigen::MatrixXd& A, Eigen::VectorXd& FT, Eigen::Vector3d accel_local,
					 Eigen::Vector3d avel_local, Eigen::Vector3d aaccel_local, Eigen::Vector3d g_local, Eigen::VectorXd& phi, Eigen::MatrixXd& Sigma, const int filter_size)
{	

	if (n_measurements == 0)
	{

		Sigma = Eigen::MatrixXd::Identity(10,10);

	}
	else if (n_measurements <= filter_size)
	{	
		addMeasurementFT(n_measurements, force_virtual, A, FT, accel_local, avel_local, aaccel_local, g_local, filter_size);

	}

	else if (n_measurements == filter_size+1)

	{	
		Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(6*filter_size, 6*filter_size);
		Eigen::MatrixXd K = computeK(Sigma, A, Lambda);
		Sigma = computeSigma(K, Sigma, A);
		phi = phi + K*(FT - A*phi);


		std::cout << "phi:  " << phi.transpose() << "\n";

	}


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
