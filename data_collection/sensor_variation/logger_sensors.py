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


file.write ('linear acceleration\t joint angles\t angular velocity\n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";
JOINT_ANGLES_DATA_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";


# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period


	accel = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
	q = json.loads(r_server.get(JOINT_ANGLES_DATA_KEY).decode("utf-8"))
	avel = json.loads(r_server.get(GYROSCOPE_DATA_KEY).decode("utf-8"))




	line = " ".join([str(x) for x in accel]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in q]) + '\t' +\
	'\n'


	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
