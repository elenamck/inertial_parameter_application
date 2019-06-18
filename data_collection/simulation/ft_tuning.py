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
folder = 'tuning/'
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
# file.write ('timestamp\tjoint pos virtual\tjoint vel virtual\tvirtual force\t accel\t avel\t aaccel\t virtual controller gain\n')
file.write ('joint pos virtual\ttvirtual force\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
JOINT_POS_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_q";
JOINT_VEL_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_dq";
SIM_FLAG_KEY = "sai2::DemoApplication::Panda::simulation::loop";
SIM_TIMESTAMP_KEY = "sai2::DemoApplication::Panda::simulation::timestamp";
GAIN_VIRTUAL_SENSOR_KEY = "sai2::DemoApplication::Panda::simulation::sensor_gains";
FORCE_VIRTUAL_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

LINEAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";
ANGULAR_VEL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
ANGULAR_ACC_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";

# gains = json.loads(r_server.get(GAIN_VIRTUAL_SENSOR_KEY).decode("utf-8"))
# gains_line =' '.join([str(x) for x in gains]) 
# file.write  ('\t' + gains_line );


# data logging frequency
logger_frequency = 3000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

while(runloop):
    sim_loop_key = json.loads(r_server.get(SIM_FLAG_KEY))
    if(sim_loop_key == 0):


        print('Waiting for flag ... \n')
    elif (sim_loop_key == 1):
        print('Start Logging Data ... \n')

        t += logger_period

        # sim_time = json.loads(r_server.get(SIM_TIMESTAMP_KEY).decode("utf-8"))
        q      = json.loads(r_server.get(JOINT_POS_VIRTUAL_KEY).decode("utf-8"))
        # dq       = json.loads(r_server.get(JOINT_VEL_VIRTUAL_KEY).decode("utf-8"))
        ft_virtual = json.loads(r_server.get(FORCE_VIRTUAL_KEY).decode("utf-8"))

        # line =  \
        # str(sim_time) + '\t' + \
        # " ".join([str(x) for x in q]) + '\t' + \
        # " ".join([str(x) for x in dq]) + '\t' +\
        # " ".join([str(x) for x in ft_virtual]) + '\t' + \
        # '\n'
        line =  \
        " ".join([str(x) for x in q]) + '\t' + \
        " ".join([str(x) for x in ft_virtual]) + '\t' + \
        '\n'

        file.write(line)

        counter = counter + 1

        time.sleep(max(0.0,t-time.time()))
    else:
            print("Error in reading sim flag key! Key is: {} ".format(sim_loop_key))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Data file    : ", filename)

file.close()

    
