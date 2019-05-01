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

const string robot_file = "../resources/02-utilities/panda_arm.urdf";
const std::string robot_name = "FRANKA-PANDA";

#define  FIRST_MEAS		 0
#define  SECOND_MEAS	 1
#define  THIRD_MEAS		 2
#define  FOURTH_MEAS	 4
#define  FITH_MEAS		 5
#define	 SIXTH_MEAS		 6
#define  SEVENTH_MEAS	 7
#define  DONE			 8



unsigned long long controller_counter = 0;

// const bool flag_simulation = true;
const bool flag_simulation = false;

const bool inertia_regularization = true;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_ANGLES_DES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_FORCE_KEY;
std::string EE_FORCE_SENSOR_KEY;


// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;


int main() {
	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		JOINT_ANGLES_DES_KEY  = "sai2::DemoApplication::Panda::controller::qdes";

		EE_FORCE_SENSOR_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";


	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		JOINT_ANGLES_DES_KEY  = "sai2::FrankaPanda::Clyde:::controller::qdes";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";	
	}

	//writing data file 

	ofstream myfile;
  	myfile.open ("../../02-utilities/FT_data1.txt");

  	// myfile.open ("../../data_collection/FINAL_DATA/FT_Calib/hardware/bias_fixed_cables.txt");

  	VectorXd force_moment_sum = VectorXd::Zero(6);
  	VectorXd force_moment_average = VectorXd::Zero(6);
  	VectorXd force_moment_average_temp = VectorXd::Zero(6);





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


	// joint controller
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_max_velocity = 0.3;

	joint_task->_kp = 80.0;
	joint_task->_kv = 25.0;
	joint_task->_ki = 30.0;
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	double n = 0; // variable to wait 2 sec in pose

	// VectorXd desired_initial_configuration = VectorXd::Zero(dof);
	//desired_initial_configuration << 0, 0, 0, 0, 0, 0, 0;
	//desired_initial_configuration << -50,  15, 35, -80, -14, 90, 130; //last
	//desired_initial_configuration << -40,  14, 25, -80, 166, 90, -138;
	//desired_initial_configuration << -0.702242,0.245541,0.438873,-1.40453,2.88915,1.57993,-2.4062; //in rad
	// desired_initial_configuration <<-0.212803,-0.894344,-0.323602,-2.45985,-0.240155,1.59861,2.3368;
	//desired_initial_configuration <<-0.704383,0.247327,0.410896,-1.40101,-0.25589,1.58301, 2.40276;

	//desired_initial_configuration *= M_PI/180.0;
	   VectorXd q_des_degrees = VectorXd::Zero(dof);

        q_des_degrees << -90, 30, 90, -90, -30, 90, 0;
   		VectorXd q_desired_1 = M_PI/180.0 * q_des_degrees;

        q_des_degrees << -90, 30, 90, -90, 60, 90, -135;
		VectorXd q_desired_2 = M_PI/180.0 * q_des_degrees;

        q_des_degrees << -90, 30, 90, -90, 60, 90, -45;
        VectorXd q_desired_3 = M_PI/180.0 * q_des_degrees;

        q_des_degrees << -90, 30, 90, -90, 60, 90, 45;
        VectorXd q_desired_4 = M_PI/180.0 * q_des_degrees;

        q_des_degrees << -90, 30, 90, -90, 60, 90, 135;
        VectorXd q_desired_5 = M_PI/180.0 * q_des_degrees;

        q_des_degrees << -90, 30, 90, -90, 150, 90, 0;
        VectorXd q_desired_6 = M_PI/180.0 * q_des_degrees; 

	joint_task->_goal_position = q_desired_1;

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
		force_moment = redis_client.getEigenMatrixJSON(EE_FORCE_SENSOR_FORCE_KEY);

		
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



			VectorXd config_error = joint_task->_goal_position- joint_task->_current_position;

			cout << "config error norm" << config_error.norm() << endl;
			if(config_error.norm() <= 0.02)
			{
				
				n++;
				if (n>=3000 && n<=4000)
				{	
					force_moment_sum += force_moment;
				}
				if(n==6000)

				{
					force_moment_average = force_moment_sum / 1000;
					std::cout << "average1" << force_moment_average << "\n";
					force_moment_sum = VectorXd::Zero(6);

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

			// joint_task->_goal_position(4) = desired_initial_configuration(4) + M_PI/2.0; 
			joint_task->_goal_position= q_desired_2;

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.02)
			{
				n++;

				if (n>=3000 && n<=4000)
				{	
					force_moment_sum += force_moment;
				}
				if(n==6000)

				{
					force_moment_average_temp = force_moment_sum / 1000;
					std::cout << "average2" << force_moment_average_temp << "\n";
					force_moment_average += force_moment_average_temp;
					force_moment_sum = VectorXd::Zero(6);
					joint_task->reInitializeTask();
					state = THIRD_MEAS;
					n=0;
				}
			}
		}
		if(state == THIRD_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position = q_desired_3;

			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.02)
			{
				n++;

				if (n>=3000 && n<=4000)
				{	
					force_moment_sum += force_moment;
				}
				if(n==6000)

				{
					force_moment_average_temp = force_moment_sum / 1000;
					std::cout << "average2" << force_moment_average_temp << "\n";
					force_moment_average += force_moment_average_temp;
					force_moment_sum = VectorXd::Zero(6);
					joint_task->reInitializeTask();
					state = FOURTH_MEAS;
					n=0;
				}
			}



		}

		if(state == FOURTH_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position = q_desired_4;


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.02)
			{
				n++;

				if (n>=3000 && n<=4000)
				{	
					force_moment_sum += force_moment;
				}
				if(n==6000)

				{
					force_moment_average_temp = force_moment_sum / 1000;
					std::cout << "average2" << force_moment_average_temp << "\n";
					force_moment_average += force_moment_average_temp;
					force_moment_sum = VectorXd::Zero(6);
					joint_task->reInitializeTask();
					state = FITH_MEAS;
					n=0;
				}
			}


		}
		if(state == FITH_MEAS)
		{
			// update tasks models
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_goal_position = q_desired_5;


			// compute task torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;



			VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;
			if(config_error.norm() <= 0.02)
			{
				n++;

				if (n>=3000 && n<=4000)
				{	
					force_moment_sum += force_moment;
				}
				if(n==6000)

				{
					force_moment_average_temp = force_moment_sum / 1000;
					std::cout << "average2" << force_moment_average_temp << "\n";
					force_moment_average += force_moment_average_temp;
					force_moment_sum = VectorXd::Zero(6);
					joint_task->reInitializeTask();
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

		joint_task->_goal_position = q_desired_6;

		// compute task torques
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques + coriolis;


		VectorXd config_error = joint_task->_goal_position - joint_task->_current_position;

		if(config_error.norm() <= 0.02)
		{
			n++;
			if (n>=3000 && n<=4000)
			{	
				cout<<"config error norm"<< config_error.norm()<<endl;
				force_moment_sum += force_moment;
			}
			if(n==6000)

			{
				force_moment_average_temp = force_moment_sum / 1000;
				std::cout << "average2" << force_moment_average_temp << "\n";
				force_moment_average += force_moment_average_temp;
				force_moment_sum = VectorXd::Zero(6);
				joint_task->reInitializeTask();
				state = DONE;
				n=0;
			}
		}
	}
		if(state == DONE)
		{
			force_moment_average = force_moment_average/6;
			std::cout << "calculated bias: " << force_moment_average.transpose() << "\n";

			myfile << force_moment_average; 
			myfile.close();	
			break;

		}

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, joint_task->_current_position);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, joint_task->_current_velocity);
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_DES_KEY, joint_task->_goal_position);

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