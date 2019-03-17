#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/KalmanFilter.h"
#include "filters/QuaternionBasedEKF.h"


#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>
#include <vector>
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
std::string LINEAR_ACCELERATION_LOCAL_KEY;
std::string ANGULAR_VELOCITY_LOCAL_KEY;
std::string EE_FORCE_SENSOR_KEY;
std::string QUATERNION_KEY;
std::string POSITION_KEY;
std::string KALMAN_FILTER_POS_KEY;
std::string KALMAN_FILTER_VEL_KEY;
std::string KALMAN_FILTER_ACC_KEY;
std::string E_KALMAN_FILTER_ORI_KEY;
std::string E_KALMAN_FILTER_AVEL_KEY;
std::string E_KALMAN_FILTER_AACC_KEY;


int main() {

	LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
	ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
	EE_FORCE_SENSOR_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";
	QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
	POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";

	KALMAN_FILTER_POS_KEY  = "sai2::DemoApplication::KF::position";
	KALMAN_FILTER_VEL_KEY  = "sai2::DemoApplication::KF::velocity";
	KALMAN_FILTER_ACC_KEY  = "sai2::DemoApplication::KF::acceleration";

	E_KALMAN_FILTER_ORI_KEY  = "sai2::DemoApplication::EKF::orientation";
	E_KALMAN_FILTER_AVEL_KEY = "sai2::DemoApplication::EKF::angular_velocity";
	E_KALMAN_FILTER_AACC_KEY = "sai2::DemoApplication::EKF::angular_acceleration";

    VectorXd force_moment = VectorXd::Zero(6);    
    //Kalman Filter
    int n_kf = 9;
    int m_kf = 6;
    double dt = 1.0/1000;
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
    Vector3d kf_pos = Vector3d::Zero();
    Vector3d kf_vel = Vector3d::Zero();
    Vector3d kf_accel = Vector3d::Zero();
    
    
    
    //Extended Kalman Filter
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
    VectorXd ekf_ori = VectorXd::Zero(4);
    Vector3d ekf_avel = Vector3d::Zero();
    Vector3d ekf_aaccel = Vector3d::Zero();
    // start redis client
    auto redis_client = RedisClient();
    redis_client.connect();
    
    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    
    // create a loop timer
    double control_freq = 1000;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);   // 1 KHz
    // timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
    timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
    timer.initializeTimer(1000000); // 1 ms pause before starting loop

	ifstream sensor_data;
    sensor_data.open("data_file_03-15-19_11-34-21.txt");
    if (!sensor_data)
    {                     // if it does not work
        std::cout << "Can't open Data!\n";
    }
    VectorXd sensor_data_read = VectorXd::Zero(19);
    std::vector<double> data_vector;
    double data = 0.0;
    // while window is open:
    while(!sensor_data.eof())
    {
	    while (runloop) 
	   	{    
	        	timer.waitForNextLoop();

	       		for(int i=0; i<19; i++)
	        	{
	        		sensor_data >> data;
	         		sensor_data_read(i) = data;
	         	}
	         	current_position_aux << sensor_data_read(0), sensor_data_read(1), sensor_data_read(2);
	         	accel_aux << sensor_data_read(3), sensor_data_read(4), sensor_data_read(5);
	         	q_eff_aux << sensor_data_read(6), sensor_data_read(7),sensor_data_read(8), sensor_data_read(9);
	         	avel_aux << sensor_data_read(10), sensor_data_read(11),sensor_data_read(12);
	         	force_moment << sensor_data_read(13), sensor_data_read(14),sensor_data_read(15), sensor_data_read(16), sensor_data_read(17),sensor_data_read(18);

	         	y << current_position_aux, accel_aux;
	         	kalman_filter->update(y);
	         	kf_states = kalman_filter->state();
	         	kf_pos << kf_states(0), kf_states(1), kf_states(2);
	         	kf_vel << kf_states(3), kf_states(4), kf_states(5);
	         	kf_accel << kf_states(6), kf_states(7), kf_states(8);


	         	y_ekf << q_eff_aux, avel_aux;
				extended_kalman_filter-> update(y_ekf);
				ekf_states = extended_kalman_filter->state();
				ekf_ori << ekf_states(0), ekf_states(1), ekf_states(2), ekf_states(3);
				ekf_avel << ekf_states(4), ekf_states(5), ekf_states(6);
				ekf_aaccel << ekf_states(7), ekf_states(8), ekf_states(9);


	        	redis_client.setEigenMatrixDerived(LINEAR_ACCELERATION_LOCAL_KEY, accel_aux);
				redis_client.setEigenMatrixDerived(ANGULAR_VELOCITY_LOCAL_KEY, avel_aux);
				redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, force_moment);
				redis_client.setEigenMatrixDerived(QUATERNION_KEY, q_eff_aux);
				redis_client.setEigenMatrixDerived(POSITION_KEY, current_position_aux);

				redis_client.setEigenMatrixDerived(KALMAN_FILTER_POS_KEY, kf_pos);
				redis_client.setEigenMatrixDerived(KALMAN_FILTER_VEL_KEY, kf_vel);
				redis_client.setEigenMatrixDerived(KALMAN_FILTER_ACC_KEY, kf_accel);

				redis_client.setEigenMatrixDerived(E_KALMAN_FILTER_ORI_KEY, ekf_ori);
				redis_client.setEigenMatrixDerived(E_KALMAN_FILTER_AVEL_KEY, ekf_avel);
				redis_client.setEigenMatrixDerived(E_KALMAN_FILTER_AACC_KEY, ekf_aaccel);
	              
	        	controller_counter++; 
	        	if (sensor_data.eof())
				{
					cout << "end of data file!" << endl; 
				}
	    	}

		}

    sensor_data.close();
    
    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    
    return 0;
    
}