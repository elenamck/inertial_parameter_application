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
folder = 'frame_calibration/'
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
file.write ('accelerometer data\t local gravity  \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)


ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";
LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::FrankaPanda::Clyde::g_local";
BASE_GRAVITY_KEY =  "sai2::DemoApplication::FrankaPanda::Clyde::g_base";



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period


	g_accelerometer = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
	g_local = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))
	g_base = json.loads(r_server.get(BASE_GRAVITY_KEY).decode("utf-8"))



	
	#test = json.loads(r_server.get("test").decode("utf-8"))

	line = " ".join([str(x) for x in g_accelerometer]) + '\t' +\
	" ".join([str(x) for x in g_local]) + '\t' +\
	" ".join([str(x) for x in g_base]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
