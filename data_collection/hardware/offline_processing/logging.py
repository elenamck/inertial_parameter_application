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
filename = name + '_' + timestamp

# all kinematic variables in frame of last link
file.write ('position\t  accelerometer gravity removed\t quaternion\t  gyroscope\t force/torque sensor \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
QUATERNION_KEY = "sai2::DemoApplication::Panda::controller::quaternion";
POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";
EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::FrankaPanda::controller::force_moment";




# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period

	pos     = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
	accel   = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))
	ori     = json.loads(r_server.get(QUATERNION_KEY).decode("utf-8"))
	avel    = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	ft      = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))


	line = " ".join([str(x) for x in pos]) + '\t' +\
	" ".join([str(x) for x in accel]) + '\t' +\
	" ".join([str(x) for x in ori]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in ft]) + '\t' +\
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