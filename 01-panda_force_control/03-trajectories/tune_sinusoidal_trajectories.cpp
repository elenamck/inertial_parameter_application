#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "Sai2Primitives.h"
#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>
#include <random>

#include "timer/LoopTimer.h"
#include "trajectories/JointSpaceSinusodial.h"
#include "parameter_estimation/LeastSquare.h"




#include <signal.h>
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

// redis keys:
// - write:
string JOINT_TORQUES_COMMANDED_KEY;
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string EE_FORCE_SENSOR_FORCE_KEY;



// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

bool check_limits_max(Eigen::VectorXd& to_test, Eigen::VectorXd& lim_max, int coeffs);
bool check_limits_min(Eigen::VectorXd& to_test, Eigen::VectorXd& lim_min, int coeffs);



int main() {
	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
		EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
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

	Vector3d position = Vector3d::Zero();
	int dof = robot->dof();
	// VectorXd command_torques = VectorXd::Zero(dof);
	// VectorXd coriolis = VectorXd::Zero(dof);
	// MatrixXd N_prec = MatrixXd::Identity(dof,dof);



	std::random_device rd;
	static std::mt19937 gen(rd());  //here you could also set a seed
	// static std::normal_distribution<double> dis(0.0,1.0);
	static std::uniform_real_distribution<double> dis(-0.8, 0.8);

	int axis = 4; 
	int N = 3; 
	double w_s = 500;
	double w_f = 0.8; 
	VectorXd a = VectorXd::Zero(N*axis);
	VectorXd b = VectorXd::Zero(N*axis);

	a = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});
	b = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});

	VectorXd q_lim_max_all = VectorXd::Zero(7);
	VectorXd q_lim_min_all = VectorXd::Zero(7);
	// q_lim_max_all <<  2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973;	
	// q_lim_min_all << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

	q_lim_max_all <<  2.7,  1.6,  2.7, 0.1,  2.7,  3.6,  2.7;	
	q_lim_min_all << -2.7, -1.6, -2.7, -2.9, -2.7, 0.1, -2.7;
	VectorXd dq_lim_max_all = VectorXd::Zero(7);
	dq_lim_max_all <<  2.1750, 2.1750, 2.1750, 2.1750,  2.6100, 2.6100,  2.6100;	

	VectorXd ddq_lim_max_all = VectorXd::Zero(7);
	ddq_lim_max_all <<  15, 7.5, 10, 12.5,  15, 20,  20;	

	VectorXd q_lim_max = VectorXd::Zero(axis);
	VectorXd q_lim_min = VectorXd::Zero(axis);
	VectorXd dq_lim_max = VectorXd::Zero(axis);
	VectorXd ddq_lim_max = VectorXd::Zero(axis);

	q_lim_max = q_lim_max_all.tail(axis);
	q_lim_min = q_lim_min_all.tail(axis);
	dq_lim_max = dq_lim_max_all.tail(axis);
	ddq_lim_max  = ddq_lim_max_all.tail(axis);

	VectorXd desired_initial_configuration_all = VectorXd::Zero(7);
	desired_initial_configuration_all << 0,  -45, 0, -115, 0, 60, 60;
	desired_initial_configuration_all *= M_PI/180.0;
	VectorXd desired_initial_configuration = desired_initial_configuration_all.tail(axis);

	unsigned long trajectory_counter = 0;
	auto joint_trajectory = new Trajectories::JointSpaceSinusodial(axis, N, w_s, w_f, a,b);
	joint_trajectory->init(desired_initial_configuration);
	VectorXd q = VectorXd::Zero(axis);
	VectorXd dq = VectorXd::Zero(axis);
	VectorXd ddq = VectorXd::Zero(axis);


	auto least_square = new ParameterEstimation::LeastSquare(false);
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0,0,0.15);
	MatrixXd Corr_mat = MatrixXd::Zero(6,6);

	Vector3d accel_local = Vector3d::Zero(); // object linear acceleration in sensor frame
	Vector3d aaccel_local = Vector3d::Zero(); // object angular acceleration in sensor frame
	Vector3d avel_local = Vector3d::Zero(); //object angular velocity in sensor frame
	Vector3d g_local = Vector3d::Zero(); //gravity vector in sensor frame
	MatrixXd A_data = MatrixXd::Zero(6,10); //Data matrix
	Matrix3d R_link = Matrix3d::Zero();
	// create a loop timer
	double control_freq = 500;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	ofstream trajectory_file;
  	trajectory_file.open ("../../../01-panda_force_control/03-trajectories/sinusoidal_trajectories_03.txt");

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		joint_trajectory->update(trajectory_counter);
		q = joint_trajectory->getJointAngles();
		dq = joint_trajectory->getJointVelocities();
		ddq = joint_trajectory->getJointAccelerations();

		trajectory_counter++;

		//check conditions
		bool conditions = (check_limits_max(q, q_lim_max, axis)) || (check_limits_min(q, q_lim_min, axis)) || (check_limits_max(dq, dq_lim_max, axis)) || (check_limits_max(ddq, ddq_lim_max, axis)) || (q(axis-2) < q_lim_min(axis-2));
		// if((q(axis-2)< -0.0175)||(q(axis-2)> 3.7525) || (q(axis-4)< -3.0718)||(q(axis-4)>-0.0698))
		// {
		// 	conditions = true;
		// 	cout << "there was a bug!" << endl;
		// }

		if(conditions == true)
		{	
			cout << "the limits are reached!" << endl;	
			
			a = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});
			b = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});
			trajectory_counter = 0;
			joint_trajectory->init(desired_initial_configuration,a,b);
			least_square->initConditioning();
		}
		robot->_q = q;
		robot->_dq = dq;
		robot->updateModel();
		robot->linearAcceleration(accel_local,link_name,pos_in_link);
		robot->rotation(R_link,link_name);
		robot->angularVelocity(avel_local,link_name);
		robot->angularAcceleration(aaccel_local,link_name);	
		g_local = R_link.transpose()*robot->_world_gravity;
		accel_local = R_link.transpose()*accel_local;	
		accel_local += g_local;
		avel_local = R_link.transpose()*avel_local;
		aaccel_local = R_link.transpose()*aaccel_local;
		least_square->addDataConditioning(accel_local,avel_local, aaccel_local, g_local);

		//check if trajectory finished
		if((trajectory_counter/w_s) >= 2*M_PI/w_f)
		{

			cout << "trajectory finished with coeffs a: \n" << endl;
			for (int i=0; i<axis*N; i++)
			{
				cout << a(i)<< ", \t";
			}
			
			cout << "\n and coeffs b: \n" << endl;

			for (int i=0; i<axis*N; i++)
			{
				cout << b(i)<< ", \t";
			}
			Corr_mat = least_square->getCorrelationMatrixConditioning(); 
			double cond = Corr_mat.completeOrthogonalDecomposition().pseudoInverse().norm() * Corr_mat.norm();
			cout << "the corresponding condition number is: " << cond << endl;


			trajectory_file << "a: \n" << a.transpose() << "\n"; 
			trajectory_file << "b: \n" << b.transpose() << "\n";
			trajectory_file << "kappa: \n" << cond << "\n";


			a = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});
			b = VectorXd::NullaryExpr((N*axis),[&](){return dis(gen);});
			trajectory_counter = 0;
			joint_trajectory->init(desired_initial_configuration,a,b);
			least_square->initConditioning();

			
		}

	}

    //command_torques << 0,0,0,0,0,0,0;
    //redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	trajectory_file.close();
    double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Loop run time  : " << end_time << " seconds\n";
    cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;

}

bool check_limits_max(Eigen::VectorXd& to_test, Eigen::VectorXd& lim_max, int coeffs)
{	
	bool is_bigger = false;

	for(int i = 0; i < coeffs; i++)
	{
		if(to_test(i)>lim_max(i))
		{
			is_bigger = true;
			break;
		}	

	}


	return is_bigger;
}

bool check_limits_min(Eigen::VectorXd& to_test, Eigen::VectorXd& lim_min, int coeffs)
{
	bool is_smaller = false;


	for(int i = 0; i < coeffs; i++)
	{
		if(to_test(i)<lim_min(i))
		{
			is_smaller = true;

			break;
		}	

	}
	if(to_test(coeffs-2) < lim_min(coeffs-2))
	{
		is_smaller = true;
	}


	return is_smaller;	
}

