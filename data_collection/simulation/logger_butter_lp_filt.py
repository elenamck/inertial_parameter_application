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
folder = 'butter_lp_filt/'
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
file.write (' accel_lp\t avel_lp\t aaccel_lop\t  accel_butter\t avel_butter\t accel_sensor\t avel_sensor\t ft_sensor \t ft_butter \t ft_lp \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)




ACCELEROMETER_DATA_KEY = "sai2::3spaceSensor::data::accelerometer";      
GYROSCOPE_DATA_KEY ="sai2::3spaceSensor::data::gyroscope";   


LINEAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel::lowpass";
ANGULAR_VEL_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel::lowpass";
ANGULAR_ACC_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel::lowpass";


LINEAR_ACC_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel::butter";
ANGULAR_VEL_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel::butter";

EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
EE_FORCE_SENSOR_LP_KEY = "sai2::DemoApplication::FrankaPanda::controller::force_moment::lowpass";
EE_FORCE_SENSOR_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::controller::force_moment::butter";

INERTIAL_PARAMS_LP_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::lowpass";
INERTIAL_PARAMS_BUTTER_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::butter";
INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::estimation::inertial_parameter::phi";









# data logging frequency
logger_frequency = 1000  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    # pos       = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
    # vel       = json.loads(r_server.get(LINEAR_VEL_KEY).decode("utf-8"))

    accel_lp     = json.loads(r_server.get(LINEAR_ACC_LP_KEY).decode("utf-8"))
    avel_lp      = json.loads(r_server.get(ANGULAR_VEL_LP_KEY).decode("utf-8"))
    aaccel_lp    = json.loads(r_server.get(ANGULAR_ACC_LP_KEY).decode("utf-8"))
    phi_lp       = json.loads(r_server.get(INERTIAL_PARAMS_LP_KEY).decode("utf-8"))
    accel_butter     = json.loads(r_server.get(LINEAR_ACC_KEY).decode("utf-8"))
    avel_butter      = json.loads(r_server.get(ANGULAR_VEL_KEY).decode("utf-8"))
    phi_butter       = json.loads(r_server.get(INERTIAL_PARAMS_BUTTER_KEY).decode("utf-8"))

    accel_sensor     = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
    avel_sensor      = json.loads(r_server.get(GYROSCOPE_DATA_KEY).decode("utf-8"))

    ft_lp = json.loads(r_server.get(EE_FORCE_SENSOR_LP_KEY).decode("utf-8"))
    ft_butter = json.loads(r_server.get(EE_FORCE_SENSOR_BUTTER_KEY).decode("utf-8"))
    ft_sensor = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))
    phi_nofilt       = json.loads(r_server.get(INERTIAL_PARAMS_KEY).decode("utf-8"))






    line = " ".join([str(x) for x in accel_lp]) + '\t' +\
    " ".join([str(x) for x in avel_lp]) + '\t' +\
    " ".join([str(x) for x in aaccel_lp]) + '\t' +\
    " ".join([str(x) for x in ft_lp]) + '\t' +\
    " ".join([str(x) for x in phi_lp]) + '\t' +\
    " ".join([str(x) for x in accel_butter]) + '\t' +\
    " ".join([str(x) for x in avel_butter]) + '\t' +\
    " ".join([str(x) for x in ft_butter]) + '\t' +\
    " ".join([str(x) for x in phi_butter]) + '\t' +\
    " ".join([str(x) for x in accel_sensor]) + '\t' +\
    " ".join([str(x) for x in avel_sensor]) + '\t' +\
    " ".join([str(x) for x in ft_sensor]) + '\t' +\
    " ".join([str(x) for x in phi_nofilt]) + '\t' +\
    '\n'

    # line = " ".join([str(x) for x in pos]) + '\t' +\
    # " ".join([str(x) for x in vel]) + '\t' +\
    # " ".join([str(x) for x in accel]) + '\t' +\
    # " ".join([str(x) for x in avel]) + '\t' +\
    # " ".join([str(x) for x in aaccel]) + '\t' +\
    # " ".join([str(x) for x in g_local]) + '\t' +\
    # " ".join([str(x) for x in accel_lp]) + '\t' +\
    # " ".join([str(x) for x in avel_lp]) + '\t' +\
    # " ".join([str(x) for x in aaccel_lp]) + '\t' +\
    # " ".join([str(x) for x in accel_sensor]) + '\t' +\
    # " ".join([str(x) for x in avel_sensor]) + '\t' +\
    # '\n'


    file.write(line)

    counter = counter + 1

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")
print("Data file    : ", filename)

file.close()
