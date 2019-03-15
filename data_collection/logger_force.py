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
#folder = 'debug/'
folder = 'calibration/'
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


#what it \t for????
#file.write('timestamp\tvelocity based observer\tmomentum based observer\tcontact compensation torques\t' +\
#	'command torques\tdesired position\tcurrent position\n')
#file.write ('timestamp\tcommand torques\tdesired position\tcurrent position\n')
file.write ('timestamp\t force_torque  \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)


EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force"



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period


	force_torque = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))



	
	#test = json.loads(r_server.get("test").decode("utf-8"))

	line = " ".join([str(x) for x in force_torque]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
