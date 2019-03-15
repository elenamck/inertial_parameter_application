#!/usr/bin/env python3

# read redis keys and dump them to a file
import redis, time, signal, sys
import os
import json

runloop = True
counter = 0

# handle ctrl-C and close the files
def signal_handler(signal, frame):
	global runloop
	runloop = False
	print(' ... Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'data/'
# folder = 'simulation/'
#folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "data_file"

# open files
# file = open(folder + '/' + name + '_' + timestamp,'w')
file = open(folder + '/' + name + '_' + timestamp,'w')

# all kinematic variables in frame of last link
file.write (' joint angles\t  joint velocities\t joint torques commanded\t position\t linear velocity \t linear acceleration \t quaternion \t angular velocity \t angular acceleration\t gravity  \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
## joint space
JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";


## cartesian space
### linear variables - derived from robot model
POSITION_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::pos";
LINEAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel_kinematics"; 

### angular variables - derived from robot model
QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::logging::quaternion"
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::avel_kinematics";
ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::aaccel_kinematics";

### gravity at last link
LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::FrankaPanda::Clyde::g_local";

### Sensor Keys
ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";       
GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope"; 

### Sensor signal gravity removed, last link frame 
LINEAR_ACCELERATION_DATA_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::accel"; 
ANGULAR_VELOCITY_DATA_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::avel"

### Kalman Estimates
KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::position";
KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::acceleration";

### Extended Kalman Filter Estimates
QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::quaternion";
ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_vel";
ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_acc";

# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period

	q     = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
	dq    = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))
	tau   = json.loads(r_server.get(JOINT_TORQUES_COMMANDED_KEY).decode("utf-8"))

	pos_kin   = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
	vel_kin   = json.loads(r_server.get(LINEAR_VELOCITY_KEY).decode("utf-8"))
	accel_kin = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))

	ori_kin   = json.loads(r_server.get(QUATERNION_KEY).decode("utf-8"))
	avel_kin  = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	aaccel_kin = json.loads(r_server.get(ANGULAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))

	g = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))

	accel_sensor_raw = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
	avel_sensor_raw  = json.loads(r_server.get(GYROSCOPE_DATA_KEY).decode("utf-8"))

	accel_sensor_corrected = json.loads(r_server.get(LINEAR_ACCELERATION_DATA_LOCAL_KEY).decode("utf-8"))
	avel_sensor_corrected  = json.loads(r_server.get(ANGULAR_VELOCITY_DATA_LOCAL_KEY).decode("utf-8"))

	pos_kf   = json.loads(r_server.get(KALMAN_FILTER_POS_KEY).decode("utf-8"))
	vel_kf   = json.loads(r_server.get(KALMAN_FILTER_VEL_KEY).decode("utf-8"))
	accel_kf = json.loads(r_server.get(KALMAN_FILTER_ACC_KEY).decode("utf-8"))

	ori_ekf    = json.loads(r_server.get(QUATERNION_ESTIMATE_KEY).decode("utf-8"))
	avel_ekf   = json.loads(r_server.get(ANGULAR_VEL_ESTIMATE_KEY).decode("utf-8"))
	aaccel_ekf = json.loads(r_server.get(ANGULAR_ACC_ESTIMATE_KEY).decode("utf-8"))

	line = " ".join([str(x) for x in q]) + '\t' +\
	" ".join([str(x) for x in dq]) + '\t' +\
	" ".join([str(x) for x in tau]) + '\t' +\
	" ".join([str(x) for x in pos_kin]) + '\t' +\
	" ".join([str(x) for x in vel_kin]) + '\t' +\
	" ".join([str(x) for x in accel_kin]) + '\t' +\
	" ".join([str(x) for x in ori_kin]) + '\t' +\
	" ".join([str(x) for x in avel_kin]) + '\t' +\
	" ".join([str(x) for x in aaccel_kin]) + '\t' +\
	" ".join([str(x) for x in g]) + '\t' +\
	" ".join([str(x) for x in accel_sensor_raw]) + '\t' +\
	" ".join([str(x) for x in avel_sensor_raw]) + '\t' +\
	" ".join([str(x) for x in accel_sensor_corrected]) + '\t' +\
	" ".join([str(x) for x in avel_sensor_corrected]) + '\t' +\
	" ".join([str(x) for x in pos_kf]) + '\t' +\
	" ".join([str(x) for x in vel_kf]) + '\t' +\
	" ".join([str(x) for x in accel_kf]) + '\t' +\
	" ".join([str(x) for x in ori_ekf]) + '\t' +\
	" ".join([str(x) for x in avel_ekf]) + '\t' +\
	" ".join([str(x) for x in aaccel_ekf]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()