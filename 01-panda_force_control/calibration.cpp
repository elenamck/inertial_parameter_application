#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

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

const string robot_file = "../resources/01-panda_force_control/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

#define  FIRST_MEAS		 0
#define  SECOND_MEAS	 1
#define  THIRD_MEAS		 2
#define  FOURTH_MEAS	 4
#define  FITH_MEAS		 5
#define	 SIXTH_MEAS		 6
#define  SEVENTH_MEAS	 7



unsigned long long controller_counter = 0;

const bool flag_simulation = true;
//const bool flag_simulation = false;

const bool inertia_regularization = true;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

void logData(const VectorXd& force_moment);

int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::force_sesnor::force_moment";
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


	////////////////////////////////////////////////
	///        Prepare the controllers         /////
	////////////////////////////////////////////////

	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);
	VectorXd force_moment = VectorXd::Zero(6);

	// pos ori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.1);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_max_velocity = 0.08;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;


	posori_task->_velocity_saturation = true;
	posori_task->_linear_saturation_velocity = Eigen::Vector3d(0.1,0.1,0.1);
	posori_task->_angular_saturation_velocity = Eigen::Vector3d(M_PI/5.0, M_PI/5.0, M_PI/5.0);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	Matrix3d initial_orientation = Matrix3d::Zero();

	// joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = 0.3;

	joint_task->_kp = 100.0;
	joint_task->_kv = 17.0;
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	double n = 0; // variable to wait 2 sec in pose

	VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	//desired_initial_configuration << 0, 0, 0, 0, 0, 0, 0;
	desired_initial_configuration << -50,  15, 35, -80, -10, 90, 130;


	desired_initial_configuration *= M_PI/180.0;
	joint_task->_goal_position = desired_initial_configuration;

	int state = FIRST_MEAS;

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
		if(state == FIRST_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = desired_initial_configuration - joint_task->_current_position;
		
			if(config_error.norm() <= 0.47)
			{
				n++;
				if(n==5000)
				{
				redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, force_moment);
				logData(force_moment);
				joint_task->reInitializeTask();
				state = SECOND_MEAS;
				n=0;
			}

			}
		

		}
		if(state == SECOND_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position(4) = desired_initial_configuration(4) + M_PI/2.0; 


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.25)
			{
				n++;
				if (n==5000)
				{
					joint_task->reInitializeTask();
					state = THIRD_MEAS;
					std::cout << "State Transition2" << "\n";
					n=0;
				}
			}
		}
		if(state == THIRD_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position(6) = desired_initial_configuration(6) - M_PI/2.0; 


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.25)
			{
				n++;
				if(n==5000)
				{	
					joint_task->reInitializeTask();
					robot->rotation(initial_orientation, link_name);
					state = FOURTH_MEAS;
					std::cout << "State Transition3" << "\n";
					n=0;
				}
			}



		}

		if(state == FOURTH_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position(6) = desired_initial_configuration(6) - M_PI; 


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.25)
			{
				n++;
				if(n==5000)
				{
					joint_task->reInitializeTask();
					robot->rotation(initial_orientation, link_name);
					std::cout << "State Transition4" << "\n";
					state = FITH_MEAS;
					n = 0;
				}
			}


		}
		if(state == FITH_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position(6) = desired_initial_configuration(6) - 3*M_PI/2.0; 


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.25)
			{
				n++;
				if(n==5000)
				{
					joint_task->reInitializeTask();
					std::cout << "State Transition5" << "\n";
					state = SIXTH_MEAS;
					n=0;
				}
			}


		}


		
		if(state == SIXTH_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position(4) = desired_initial_configuration(4) + M_PI; 

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.25)
			{
				n++;
				if(n==5000)
				{
				joint_task->reInitializeTask();
				}
			}
		}

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

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
void logData(const VectorXd& force_moment)
{
 ofstream myfile;
  myfile.open ("FT_data.txt");
  if (myfile.is_open())
  {
  	std::cout << "file is open"; 
  myfile << force_moment << endl;
  } else
  {
  	std::cout << "error opening file"; 
  }
  myfile.close();
 }