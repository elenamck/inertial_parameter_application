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
folder = 'inertial_params_est/'
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
file.write ('pos\t vel\t accel\t ori(matrix)\t avel\t aaccel\t force \t phi RLS\t phi LS \t phi debug \n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys
POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
LINEAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::accel";
ORIENTATION_QUATERNION_KEY =  "sai2::DemoApplication::Panda::kinematics::ori::quats";
ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::avel";
ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::aaccel";

EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";

JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
# INERTIAL_PARAMS_LS_KEY = "sai2::DemoApplication::Panda::controller::phiLS";
# INERTIAL_PARAMS_DEBUG_KEY = "sai2::DemoApplication::Panda::controller::phidebug";

# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    pos       = json.loads(r_server.get(POSITION_KEY).decode("utf-8"))
    vel       = json.loads(r_server.get(LINEAR_VEL_KIN_KEY).decode("utf-8"))
    accel     = json.loads(r_server.get(LINEAR_ACC_KIN_KEY).decode("utf-8"))
    ori_quat  = json.loads(r_server.get(ORIENTATION_QUATERNION_KEY).decode("utf-8"))
    avel      = json.loads(r_server.get(ANGULAR_VEL_KIN_KEY).decode("utf-8"))
    aaccel    = json.loads(r_server.get(ANGULAR_ACC_KIN_KEY).decode("utf-8"))
    force_v   = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))
    phi_RLS   = json.loads(r_server.get(INERTIAL_PARAMS_KEY).decode("utf-8"))
    q         = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
    dq        = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))
    q_6       = q[6];
    dq_6      = dq[6];

    #phi_LS    = json.loads(r_server.get(INERTIAL_PARAMS_LS_KEY).decode("utf-8"))
    #phi_aux   = json.loads(r_server.get(INERTIAL_PARAMS_DEBUG_KEY).decode("utf-8"))

    line = " ".join([str(x) for x in pos]) + '\t' +\
    " ".join([str(x) for x in vel]) + '\t' +\
    " ".join([str(x) for x in accel]) + '\t' +\
    " ".join([str(x) for x in ori_quat]) + '\t' +\
    " ".join([str(x) for x in avel]) + '\t' +\
    " ".join([str(x) for x in aaccel]) + '\t' +\
    " ".join([str(x) for x in force_v]) + '\t' +\
    " ".join([str(x) for x in phi_RLS]) + '\t' +\
    " ".join([str(q_6)]) + '\t' +\
    " ".join([str(dq_6)]) + '\t' +\
    '\n'
    # " ".join([str(x) for x in phi_LS]) + '\t' +\
    # " ".join([str(x) for x in phi_aux]) + '\t' +\
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
