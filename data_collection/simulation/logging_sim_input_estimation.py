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
folder = 'inertial_params_input/'
# folder = 'simulation/'
#folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name_1 = "data_file_A"
name_2 = "data_file_ft"
# open files
# file = open(folder + '/' + name + '_' + timestamp,'w')
file_1 = open(folder + '/' + name_1 + '_' + timestamp,'w')
file_2 = open(folder + '/' + name_2 + '_' + timestamp,'w')
filename_1 = name_1 + '_' + timestamp
filename_2 = name_2 + '_' + timestamp

# all kinematic variables in frame of last link
file_1.write ('data matrix \n')
file_2.write (' ft \n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
DATA_MATRIX_CURR = "sai2::DemoApplication::Panda::estimation::current_data_matrix";
FT_CURR = "sai2::DemoApplication::Panda::estimation::current_data_ft";


# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    A_data       = json.loads(r_server.get(DATA_MATRIX_CURR).decode("utf-8"))
    ft_vec       = json.loads(r_server.get(FT_CURR).decode("utf-8"))


    line_1 = " ".join([str(x).replace('[]', '') for x in A_data]) + '\t' +\
    '\n'
    line_2 = " ".join([str(x) for x in ft_vec]) + '\t' +\
    '\n'
    # " ".join([str(x) for x in phi_LS]) + '\t' +\
    # " ".join([str(x) for x in phi_aux]) + '\t' +\
    # '\n'

    file_1.write(line_1)
    file_2.write(line_2)


    counter = counter + 1

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Data file 1    : ", filename_1)
print("Data file 2    : ", filename_2)

file_1.close()
file_2.close()