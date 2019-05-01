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
folder = 'FINAL/'
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
file.write ('accel\t avel\t aaccel\t gravity\t  force_torque unbiased \t phi\t q \t dq \t q_des \t dq_des \t accel_sensor \t avel_sensor\t ft_sensor \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys

LINEAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::accel";
ANGULAR_VEL_KEY = "sai2::DemoApplication::FrankaPanda::controller::avel";
ANGULAR_ACC_KEY = "sai2::DemoApplication::FrankaPanda::controller::aaccel";
LOCAL_GRAVITY_KEY = "sai2::DemoApplication::FrankaPanda::controller::g_local";

EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
EE_FORCE_SENSOR_UNBIASED_KEY ="sai2::DemoApplication::FrankaPanda::controller::force_moment";   


INERTIAL_PARAMS_KEY = "sai2::DemoApplication::FrankaPanda::controller::phi";

JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";

JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::FrankaPanda::desired::q";
JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::FrankaPanda::desired::dq";

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
    accel     = json.loads(r_server.get(LINEAR_ACC_KEY).decode("utf-8"))
    avel      = json.loads(r_server.get(ANGULAR_VEL_KEY).decode("utf-8"))
    aaccel    = json.loads(r_server.get(ANGULAR_ACC_KEY).decode("utf-8"))
    g_local   = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))
    force_v   = json.loads(r_server.get(EE_FORCE_SENSOR_UNBIASED_KEY).decode("utf-8"))
    phi_RLS   = json.loads(r_server.get(INERTIAL_PARAMS_KEY).decode("utf-8"))
    q         = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
    dq        = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))
    q_des     = json.loads(r_server.get(JOINT_ANGLE_INPUTS_KEY).decode("utf-8"))
    dq_des     = json.loads(r_server.get(JOINT_VELOCITIES_INPUTS_KEY).decode("utf-8"))
    accel_sensor = json.loads(r_server.get(ACCELEROMETER_DATA_KEY).decode("utf-8"))
    avel_sensor = json.loads(r_server.get(GYROSCOPE_DATA_KEY).decode("utf-8"))
    ft_sensor = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))


    line = " ".join([str(x) for x in accel]) + '\t \t' +\
    " ".join([str(x) for x in avel]) + '\t \t' +\
    " ".join([str(x) for x in aaccel]) + '\t \t' +\
    " ".join([str(x) for x in g_local]) + '\t \t' +\
    " ".join([str(x) for x in force_v]) + '\t \t' +\
    " ".join([str(x) for x in phi_RLS]) + '\t \t' +\
    " ".join([str(x) for x in q]) + '\t \t' +\
    " ".join([str(x) for x in dq]) + '\t \t' +\
    " ".join([str(x) for x in q_des]) + '\t \t' +\
    " ".join([str(x) for x in dq_des]) + '\t \t' +\
    " ".join([str(x) for x in accel_sensor]) + '\t \t' +\
    " ".join([str(x) for x in avel_sensor]) + '\t \t' +\
    " ".join([str(x) for x in ft_sensor]) + '\t \t' +\
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
