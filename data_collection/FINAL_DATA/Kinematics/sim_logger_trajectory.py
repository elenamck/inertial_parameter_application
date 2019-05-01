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
folder = 'debug/simulation/'
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
# file.write (' accel_lp\t avel_lp\t aaccel_lop\t  accel_butter\t avel_butter\t accel_sensor\t avel_sensor\t ft_sensor \t ft_butter \t ft_lp \n')
file.write (' pos_des\t pos_curr \t vel_des \t vel_curr \t ori_des \t ori_curr \t avel_des \t avel_curr\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)




DESIRED_POSITION_KEY = "sai2::DemoApplication::Panda::simulation::ee_pos_des";
CURRENT_POSITION_KEY = "sai2::DemoApplication::Panda::simulation::ee_pos_curr";
DESIRED_VELOCITY_KEY = "sai2::DemoApplication::Panda::simulation::ee_vel_des";
CURRENT_VELOCITY_KEY = "sai2::DemoApplication::Panda::simulation::ee_vel_curr";





CURRENT_ORIENTATION_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_ori_curr";
DESIRED_ORIENTATION_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_ori_des";
DESIRED_ANGULAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_vel_des";
CURRENT_ANGULAR_VELOCITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::ee_vel_curr";




file.write(' pos_des\t pos_curr \t vel_des \t vel_curr \t ori_des \t ori_curr \t avel_des \t avel_curr\n' )
# data logging frequency
logger_frequency = 1000  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period



    pos_des = json.loads(r_server.get(DESIRED_POSITION_KEY).decode("utf-8"))
    pos_curr = json.loads(r_server.get(CURRENT_POSITION_KEY).decode("utf-8"))

    vel_des = json.loads(r_server.get(DESIRED_VELOCITY_KEY).decode("utf-8"))
    vel_curr = json.loads(r_server.get(CURRENT_VELOCITY_KEY).decode("utf-8"))

    ori_des = json.loads(r_server.get(DESIRED_ORIENTATION_KEY).decode("utf-8"))
    ori_curr = json.loads(r_server.get(CURRENT_ORIENTATION_KEY).decode("utf-8"))

    avel_des = json.loads(r_server.get(DESIRED_ANGULAR_VELOCITY_KEY).decode("utf-8"))
    avel_curr = json.loads(r_server.get(CURRENT_ANGULAR_VELOCITY_KEY).decode("utf-8"))



    line = " ".join([str(x) for x in pos_des]) + '\t' +\
    " ".join([str(x) for x in pos_curr]) + '\t' +\
    " ".join([str(x) for x in vel_des]) + '\t' +\
    " ".join([str(x) for x in vel_curr]) + '\t' +\
    " ".join([str(x) for x in ori_des]) + '\t' +\
    " ".join([str(x) for x in ori_curr]) + '\t' +\
    " ".join([str(x) for x in avel_des]) + '\t' +\
    " ".join([str(x) for x in avel_curr]) + '\t' +\
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
