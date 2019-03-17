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
folder = 'tuning'
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
filename = name + '_'+ timestamp



file.write (' position\t linear acc\t kf position\t kf velocity\t kf acceleration\t orientation\t angular vel\t ekf orientation\t ekf angular velocity\t ekf angular acceleration\n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
KALMAN_FILTER_POS_KEY = "sai2::DemoApplication::KF::position";
KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::KF::velocity";
KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::KF::acceleration";

QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
E_KALMAN_FILTER_ORI_KEY  = "sai2::DemoApplication::EKF::orientation";
E_KALMAN_FILTER_AVEL_KEY = "sai2::DemoApplication::EKF::angular_velocity";
E_KALMAN_FILTER_AACC_KEY = "sai2::DemoApplication::EKF::angular_acceleration";



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period



	pos = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
	acc = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	kf_pos = json.loads(r_server.get(KALMAN_FILTER_POS_KEY).decode("utf-8"))
	kf_vel = json.loads(r_server.get(KALMAN_FILTER_VEL_KEY).decode("utf-8"))
	kf_acc = json.loads(r_server.get(KALMAN_FILTER_ACC_KEY).decode("utf-8"))
	ori = json.loads(r_server.get(QUATERNION_KEY).decode("utf-8"))
	avel = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	ekf_ori = json.loads(r_server.get(E_KALMAN_FILTER_ORI_KEY).decode("utf-8"))
	ekf_avel = json.loads(r_server.get(E_KALMAN_FILTER_AVEL_KEY).decode("utf-8"))
	ekf_aacel = json.loads(r_server.get(E_KALMAN_FILTER_AACC_KEY).decode("utf-8"))
	#test = json.loads(r_server.get("test").decode("utf-8"))

	line = " ".join([str(x) for x in pos]) + '\t' +\
	" ".join([str(x) for x in acc]) + '\t' +\
	" ".join([str(x) for x in kf_pos]) + '\t' +\
	" ".join([str(x) for x in kf_vel]) + '\t' +\
	" ".join([str(x) for x in kf_acc]) + '\t' +\
	" ".join([str(x) for x in ori]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in ekf_ori]) + '\t' +\
	" ".join([str(x) for x in ekf_avel]) + '\t' +\
	" ".join([str(x) for x in ekf_aacel]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Datafile.    : ", filename)

file.close()
