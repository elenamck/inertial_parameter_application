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
folder = 'debug/'
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



file.write (' angular vel\t  linear acc \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

##################################SIMULATION#####################################
# LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::accel";
# ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
# ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";

###############################HARDWARE####################################

LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::accel"
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::sensors::avel"
LINEAR_ACCELERATION_KINEMATICS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::accel_kinematics";
ANGULAR_VELOCITY_KINEMATICS_KEY = "sai2::DemoApplication::FrankaPanda::Clyde::kinematics::avel_kinematics";         



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period




	a_vel = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	a_vel_kin = json.loads(r_server.get(ANGULAR_VELOCITY_KINEMATICS_KEY).decode("utf-8"))
	l_acc = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	l_acc_kin = json.loads(r_server.get(LINEAR_ACCELERATION_KINEMATICS_KEY).decode("utf-8"))
	#test = json.loads(r_server.get("test").decode("utf-8"))

	line = " ".join([str(x) for x in a_vel]) + '\t' +\
	" ".join([str(x) for x in a_vel_kin]) + '\t' +\
	" ".join([str(x) for x in l_acc]) + '\t' +\
	" ".join([str(x) for x in l_acc_kin]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
