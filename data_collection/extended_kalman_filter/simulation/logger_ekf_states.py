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
# folder = 'simulation/'
folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "data_file"
# name = "comp_kin_kp100_fc10"

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')


file.write ('timestamp\tquat\tquat estimate\t avel\t avel estimate\t aaccel\t aaccel estimate\n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
####################################SIMULATION#############################################
SIM_TIMESTAMP_KEY = "sai2::DemoApplication::Panda::simulation::timestamp"
ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::angular_vel";
ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::angular_acc";
QUATERNION_KEY = "sai2::DemoApplication::simulation::Panda::controller::logging::quaternion";
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";
QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::simulation::Panda::estimation::quaternion";


####################################ROBOT###########################################
# SIM_TIMESTAMP_KEY = "sai2::DemoApplication::Panda::simulation::timestamp"

# QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::logging::quaternion"
# QUATERNION_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::quaternion"
# ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::avel";
# ANGULAR_VEL_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_vel"
# ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::aaccel"
# ANGULAR_ACC_ESTIMATE_KEY = "sai2::DemoApplication::Panda::estimation::angular_acc"


# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period


	sim_time = json.loads(r_server.get(SIM_TIMESTAMP_KEY).decode("utf-8"))
	q = json.loads(r_server.get(QUATERNION_KEY).decode("utf-8"))
	q_hat = json.loads(r_server.get(QUATERNION_ESTIMATE_KEY).decode("utf-8"))
	avel = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	avel_hat = json.loads(r_server.get(ANGULAR_VEL_ESTIMATE_KEY).decode("utf-8"))
	aaccel = json.loads(r_server.get(ANGULAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	aaccel_hat = json.loads(r_server.get(ANGULAR_ACC_ESTIMATE_KEY).decode("utf-8"))



	line = str(sim_time) + '\t' + \
	" ".join([str(x) for x in q]) + '\t' +\
	" ".join([str(x) for x in q_hat]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in avel_hat]) + '\t' +\
	" ".join([str(x) for x in aaccel]) + '\t' +\
	" ".join([str(x) for x in aaccel_hat]) + '\t' +\
	'\n'


	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
