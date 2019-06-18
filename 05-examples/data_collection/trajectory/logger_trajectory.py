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

# open files
# file = open(folder + '/' + name + '_' + timestamp,'w')
file = open(folder + '/' + name + '_' + timestamp,'w')
filename = name + '_' + timestamp

# all kinematic variables in frame of last link
file.write (' joint angles\t  joint velocities\t  joint acceleration \t joint torques commanded\t desired joint angles \t desired joint velocities \t desired joint acceleration  \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
## joint space
JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
JOINT_ACCELERATIONS_KEY = "sai2::DemoApplication::Panda::sensors::ddq";
JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";
JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";
JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::ddq";
# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period

	q     = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
	dq    = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))
	ddq       = json.loads(r_server.get(JOINT_ACCELERATIONS_KEY).decode("utf-8"))
	tau   = json.loads(r_server.get(JOINT_TORQUES_COMMANDED_KEY).decode("utf-8"))
	q_des     = json.loads(r_server.get(JOINT_ANGLE_INPUTS_KEY).decode("utf-8"))
	dq_des     = json.loads(r_server.get(JOINT_VELOCITIES_INPUTS_KEY).decode("utf-8"))
	ddq_des  = json.loads(r_server.get(JOINT_ACCELERATIONS_INPUTS_KEY).decode("utf-8"))

	line = " ".join([str(x) for x in q]) + '\t \t' +\
    " ".join([str(x) for x in dq]) + '\t \t' +\
    " ".join([str(x) for x in ddq]) + '\t \t' +\
    " ".join([str(x) for x in q_des]) + '\t \t' +\
    " ".join([str(x) for x in dq_des]) + '\t \t' +\
    " ".join([str(x) for x in ddq_des]) + '\t \t' +\
    " ".join([str(x) for x in tau]) + '\t \t' +\
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