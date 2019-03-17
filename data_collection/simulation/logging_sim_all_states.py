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
folder = 'all_states/'
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
file.write (' joint angles\t  joint velocities\t joint torques commanded\t position\t linear velocity \t linear acceleration \t quaternion \t angular velocity \t angular acceleration\t gravity  \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
## joint space
JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
JOINT_TORQUES_COMMANDED_KEY = "sai2::DemoApplication::Panda::actuators::fgc";


## cartesian space
### linear variables
POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
LINEAR_VELOCITY_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
LINEAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::accel";

### angular variables
QUATERNION_KEY = "sai2::DemoApplication::simulation::Panda::controller::logging::quaternion"
ANGULAR_VELOCITY_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::avel";
ANGULAR_ACCELERATION_LOCAL_KEY = "sai2::DemoApplication::Panda::sensors::aaccel";

### gravity at last link
LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";

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
	tau   = json.loads(r_server.get(JOINT_TORQUES_COMMANDED_KEY).decode("utf-8"))

	pos   = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
	vel   = json.loads(r_server.get(LINEAR_VELOCITY_KEY).decode("utf-8"))
	accel = json.loads(r_server.get(LINEAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))

	ori   = json.loads(r_server.get(QUATERNION_KEY).decode("utf-8"))
	avel  = json.loads(r_server.get(ANGULAR_VELOCITY_LOCAL_KEY).decode("utf-8"))
	aaccel = json.loads(r_server.get(ANGULAR_ACCELERATION_LOCAL_KEY).decode("utf-8"))

	g = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))

	line = " ".join([str(x) for x in q]) + '\t' +\
	" ".join([str(x) for x in dq]) + '\t' +\
	" ".join([str(x) for x in tau]) + '\t' +\
	" ".join([str(x) for x in pos]) + '\t' +\
	" ".join([str(x) for x in vel]) + '\t' +\
	" ".join([str(x) for x in accel]) + '\t' +\
	" ".join([str(x) for x in ori]) + '\t' +\
	" ".join([str(x) for x in avel]) + '\t' +\
	" ".join([str(x) for x in aaccel]) + '\t' +\
	" ".join([str(x) for x in g]) + '\t' +\
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