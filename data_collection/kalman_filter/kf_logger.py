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
# folder = 'debug/'
folder = 'kalman_filter'
# folder = 'simulation/'
#folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "data_file"
# name = "comp_kin_kp100_fc10"

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')



file.write (' position\t  linear vel\t linear acc\t kf position\t kf velocity\t kf acceleration \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

##################################SIMULATION#####################################

# POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
# LINEAR_VELOCITY_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
# LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::accel";
# KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::Panda::KF::position";
# KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::Panda::KF::velocity";
# KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::Panda::KF::acceleration";

# ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
# ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";

###############################HARDWARE####################################
POSITION_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::pos";
LINEAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::accel";
ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";
KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::position";
KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::acceleration";
LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::FrankaPanda::Clyde::g_local";
LINEAR_ACCELERATION_CORRECTED_RIGHT_FRAME_KEY ="sai2::DemoApplication::FrankaPanda::Clyde::linear_acceleration_corrected_right_frame";
# ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::avel";
# ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::aaccel";



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period



	pos = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
	vel = json.loads(r_server.get(LINEAR_VELOCITY_KEY).decode("utf-8"))
	acc = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	kf_pos = json.loads(r_server.get(KALMAN_FILTER_POS_KEY).decode("utf-8"))
	kf_vel = json.loads(r_server.get(KALMAN_FILTER_VEL_KEY).decode("utf-8"))
	kf_acc = json.loads(r_server.get(KALMAN_FILTER_ACC_KEY).decode("utf-8"))
	acc_raw= json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
	g_local = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))
	acc_g_rem = json.loads(r_server.get(LINEAR_ACCELERATION_CORRECTED_RIGHT_FRAME_KEY).decode("utf-8"))
	#test = json.loads(r_server.get("test").decode("utf-8"))

	line = " ".join([str(x) for x in pos]) + '\t' +\
	" ".join([str(x) for x in vel]) + '\t' +\
	" ".join([str(x) for x in acc]) + '\t' +\
	" ".join([str(x) for x in kf_pos]) + '\t' +\
	" ".join([str(x) for x in kf_vel]) + '\t' +\
	" ".join([str(x) for x in kf_acc]) + '\t' +\
	" ".join([str(x) for x in acc_raw]) + '\t' +\
	" ".join([str(x) for x in g_local]) + '\t' +\
	" ".join([str(x) for x in acc_g_rem]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()