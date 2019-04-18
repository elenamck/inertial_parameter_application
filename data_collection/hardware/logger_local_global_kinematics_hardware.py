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
folder = 'local_global_kinematics_hardware/'
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
file.write ('pos_local\t vel_local\t accel_local\t avel_local\t aaccel_local\t g_local \t pos_global\t vel_global\t accel_global\t avel_global\t aaccel_global\n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)


POSITION_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos";
LINEAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::controller::vel";
LINEAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
ANGULAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
ANGULAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel";
LOCAL_GRAVITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::g_local";

POSITION_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::pos_global";
LINEAR_VEL_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::vel_global";
LINEAR_ACC_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel_global";
ANGULAR_VEL_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel_global";
ANGULAR_ACC_GLOBAL_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel_global"; 


ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";      
GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";   










# data logging frequency
logger_frequency = 1000  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    pos       = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
    vel       = json.loads(r_server.get(LINEAR_VEL_KEY).decode("utf-8"))
    accel     = json.loads(r_server.get(LINEAR_ACC_KEY).decode("utf-8"))
    avel      = json.loads(r_server.get(ANGULAR_VEL_KEY).decode("utf-8"))
    aaccel    = json.loads(r_server.get(ANGULAR_ACC_KEY).decode("utf-8"))
    g_local   = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))

    pos_global       = json.loads(r_server.get(POSITION_GLOBAL_KEY).decode("utf-8"))
    vel_global       = json.loads(r_server.get(LINEAR_VEL_GLOBAL_KEY).decode("utf-8"))
    accel_global     = json.loads(r_server.get(LINEAR_ACC_GLOBAL_KEY).decode("utf-8"))
    avel_global      = json.loads(r_server.get(ANGULAR_VEL_GLOBAL_KEY).decode("utf-8"))
    aaccel_global    = json.loads(r_server.get(ANGULAR_ACC_GLOBAL_KEY).decode("utf-8"))
    accel_sensor     = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
    avel_sensor      = json.loads(r_server.get(GYROSCOPE_DATA_KEY).decode("utf-8"))


    line = " ".join([str(x) for x in pos]) + '\t' +\
    " ".join([str(x) for x in vel]) + '\t' +\
    " ".join([str(x) for x in accel]) + '\t' +\
    " ".join([str(x) for x in avel]) + '\t' +\
    " ".join([str(x) for x in aaccel]) + '\t' +\
    " ".join([str(x) for x in g_local]) + '\t' +\
    " ".join([str(x) for x in pos_global]) + '\t' +\
    " ".join([str(x) for x in vel_global]) + '\t' +\
    " ".join([str(x) for x in accel_global]) + '\t' +\
    " ".join([str(x) for x in avel_global]) + '\t' +\
    " ".join([str(x) for x in aaccel_global]) + '\t' +\
    " ".join([str(x) for x in accel_sensor]) + '\t' +\
    " ".join([str(x) for x in avel_sensor]) + '\t' +\
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
