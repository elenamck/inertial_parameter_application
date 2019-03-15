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
folder = 'inertial_params_est/'
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
filename = name + '_' + timestamp,'w'
# all kinematic variables in frame of last link
file.write (' accel\t  avel\t aaccel\t g_local\t force virtual\t phi \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::accel";
ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";
LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";
FORCE_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";
## joint space
# LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";
# ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
# ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";
# LOCAL_GRAVITY_KEY = "sai2::DemoApplication::Panda::simulation::g_local";
# FORCE_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";
# INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::simulation::inertial_parameter";




# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period




	accel   = json.loads(r_server.get(LINEAR_ACC_KEY).decode("utf-8"))
	avel    = json.loads(r_server.get(ANGULAR_VEL_KEY).decode("utf-8"))
	aaccel  = json.loads(r_server.get(ANGULAR_ACC_KEY).decode("utf-8"))
	g_local = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))
	force_v = json.loads(r_server.get(FORCE_VIRTUAL_KEY).decode("utf-8"))
	phi     = json.loads(r_server.get(INERTIAL_PARAMS_KEY).decode("utf-8"))

	line = " ".join([str(x) for x in accel]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in aaccel]) + '\t' +\
	" ".join([str(x) for x in g_local]) + '\t' +\
	" ".join([str(x) for x in force_v]) + '\t' +\
	" ".join([str(x) for x in phi]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Data file    : ", filename)

file.close()