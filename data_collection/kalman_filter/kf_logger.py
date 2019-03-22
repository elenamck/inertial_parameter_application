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
filename =  name + '_' + timestamp


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
LINEAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::vel";
LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel";
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
KALMAN_FILTER_VEL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::velocity";
KALMAN_FILTER_ACC_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::KF::acceleration";
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

	vel = json.loads(r_server.get(LINEAR_VELOCITY_KEY).decode("utf-8"))
	acc_sensed = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	acc_kin = json.loads(r_server.get(LINEAR_ACC_KIN_KEY).decode("utf-8"))
	kf_vel = json.loads(r_server.get(KALMAN_FILTER_VEL_KEY).decode("utf-8"))
	kf_acc = json.loads(r_server.get(KALMAN_FILTER_ACC_KEY).decode("utf-8"))

	line = " ".join([str(x) for x in vel]) + '\t' +\
	" ".join([str(x) for x in acc_sensed]) + '\t' +\
	" ".join([str(x) for x in acc_kin]) + '\t' +\
	" ".join([str(x) for x in kf_vel]) + '\t' +\
	" ".join([str(x) for x in kf_acc]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Filename     : ", filename)

file.close()
