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
folder = 'local_global_kinematics_simulation/'
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

# redis keys
# POSITION_KEY = "sai2::DemoApplication::Panda::kinematics::pos";
# # POS_EE_KEY = "sai2::DemoApplication::Panda::kinematics::pos::sensor";
# LINEAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::vel";
# LINEAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::accel";
# ORIENTATION_QUATERNION_KEY =  "sai2::DemoApplication::Panda::kinematics::ori::quats";
# ANGULAR_VEL_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::avel";
# ANGULAR_ACC_KIN_KEY = "sai2::DemoApplication::Panda::kinematics::aaccel";

# EE_FORCE_SENSOR_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force";

# INERTIAL_PARAMS_KEY = "sai2::DemoApplication::Panda::controller::phi";

# JOINT_ANGLES_KEY  = "sai2::DemoApplication::Panda::sensors::q";
# JOINT_VELOCITIES_KEY = "sai2::DemoApplication::Panda::sensors::dq";
# JOINT_ACCELERATIONS_KEY = "sai2::DemoApplication::Panda::sensors::ddq";


# EE_DESIRED_FORCE_KEY = "sai2::DemoApplication::Panda::simulation::virtual_force_des";

# JOINT_ANGLE_INPUTS_KEY = "sai2::DemoApplication::Panda::desired::q";
# JOINT_VELOCITIES_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::dq";
# JOINT_ACCELERATIONS_INPUTS_KEY ="sai2::DemoApplication::Panda::desired::ddq";


# LOCAL_GRAVITY_KEY =  "sai2::DemoApplication::simulation::Panda::g_local";   

# INERTIAL_PARAMS_LS_KEY = "sai2::DemoApplication::Panda::controller::phiLS";
# INERTIAL_PARAMS_DEBUG_KEY = "sai2::DemoApplication::Panda::controller::phidebug";
POSITION_SIM_KEY = "sai2::DemoApplication::Panda::simulation::position";
LINEAR_VEL_SIM_KEY = "sai2::DemoApplication::Panda::simulation::linear_vel";
LINEAR_ACC_SIM_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc";


ANGULAR_VEL_SIM_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel";
ANGULAR_ACC_SIM_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc";
LOCAL_GRAVITY_SIM_KEY = "sai2::DemoApplication::Panda::simulation::g_local";


POSITION_SIM_GLOBAL_KEY = "sai2::DemoApplication::Panda::simulation::position_global";
LINEAR_VEL_SIM_GLOBAL_KEY = "sai2::DemoApplication::Panda::simulation::linear_vel_global";
LINEAR_ACC_SIM_GLOBAL_KEY = "sai2::DemoApplication::Panda::simulation::linear_acc_global";

ANGULAR_VEL_SIM_GLOBAL_KEY = "sai2::DemoApplication::Panda::simulation::angular_vel_global";
ANGULAR_ACC_SIM_GLOBAL_KEY = "sai2::DemoApplication::Panda::simulation::angular_acc_global";






# data logging frequency
logger_frequency = 1000  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    pos       = json.loads(r_server.get(POSITION_SIM_KEY).decode("utf-8"))
    vel       = json.loads(r_server.get(LINEAR_VEL_SIM_KEY).decode("utf-8"))
    accel     = json.loads(r_server.get(LINEAR_ACC_SIM_KEY).decode("utf-8"))
    avel      = json.loads(r_server.get(ANGULAR_VEL_SIM_KEY).decode("utf-8"))
    aaccel    = json.loads(r_server.get(ANGULAR_ACC_SIM_KEY).decode("utf-8"))
    g_local   = json.loads(r_server.get(LOCAL_GRAVITY_SIM_KEY).decode("utf-8"))

    pos_global       = json.loads(r_server.get(POSITION_SIM_GLOBAL_KEY).decode("utf-8"))
    vel_global       = json.loads(r_server.get(LINEAR_VEL_SIM_GLOBAL_KEY).decode("utf-8"))
    accel_global     = json.loads(r_server.get(LINEAR_ACC_SIM_GLOBAL_KEY).decode("utf-8"))
    avel_global      = json.loads(r_server.get(ANGULAR_VEL_SIM_GLOBAL_KEY).decode("utf-8"))
    aaccel_global    = json.loads(r_server.get(ANGULAR_ACC_SIM_GLOBAL_KEY).decode("utf-8"))
    # pos_ee = json.loads(r_server.get(POS_EE_KEY).decode("utf-8")) 
    # accel_test = json.loads(r_server.get(LINEAR_ACC_TEST).decode("utf-8"))
    # avel_test = json.loads(r_server.get(ANGULAR_VEL_TEST).decode("utf-8"))
    # aaccel_test = json.loads(r_server.get(ANGULAR_ACC_TEST).decode("utf-8"))
    # avel_joint = json.loads(r_server.get(ANGULAR_VEL_JOINT).decode("utf-8"))

    #phi_LS    = json.loads(r_server.get(INERTIAL_PARAMS_LS_KEY).decode("utf-8"))
    #phi_aux   = json.loads(r_server.get(INERTIAL_PARAMS_DEBUG_KEY).decode("utf-8"))


    # accel     = json.loads(r_server.get(LINEAR_ACC_KIN_KEY).decode("utf-8"))
    # avel      = json.loads(r_server.get(ANGULAR_VEL_KIN_KEY).decode("utf-8"))
    # aaccel    = json.loads(r_server.get(ANGULAR_ACC_KIN_KEY).decode("utf-8"))
    # g_local  = json.loads(r_server.get(LOCAL_GRAVITY_KEY).decode("utf-8"))
    # force_v   = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY).decode("utf-8"))
    # phi_RLS   = json.loads(r_server.get(INERTIAL_PARAMS_KEY).decode("utf-8"))
    # q         = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
    # dq        = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))
    # ddq       = json.loads(r_server.get(JOINT_ACCELERATIONS_KEY).decode("utf-8"))
    # q_des     = json.loads(r_server.get(JOINT_ANGLE_INPUTS_KEY).decode("utf-8"))
    # dq_des     = json.loads(r_server.get(JOINT_VELOCITIES_INPUTS_KEY).decode("utf-8"))
    # ddq_des  = json.loads(r_server.get(JOINT_ACCELERATIONS_INPUTS_KEY).decode("utf-8"))
    # accel_test = json.loads(r_server.get(LINEAR_ACC_TEST).decode("utf-8"))
    # avel_test = json.loads(r_server.get(ANGULAR_VEL_TEST).decode("utf-8"))
    # aaccel_test = json.loads(r_server.get(ANGULAR_ACC_TEST).decode("utf-8"))
    # pos_global       = json.loads(r_server.get(LINEAR_ACC_KEY_SIM).decode("utf-8"))
    # accel_global     = json.loads(r_server.get(LINEAR_ACC_KEY_SIM).decode("utf-8"))
    # avel_global      = json.loads(r_server.get(ANGULAR_VEL_KEY_SIM).decode("utf-8"))
    # aaccel_global    = json.loads(r_server.get(ANGULAR_ACC_KEY_SIM).decode("utf-8"))

    # line = " ".join([str(x) for x in accel]) + '\t \t' +\
    # " ".join([str(x) for x in avel]) + '\t \t' +\
    # " ".join([str(x) for x in aaccel]) + '\t \t' +\
    # " ".join([str(x) for x in g_local]) + '\t \t' +\
    # " ".join([str(x) for x in force_v]) + '\t \t' +\
    # " ".join([str(x) for x in phi_RLS]) + '\t \t' +\
    # " ".join([str(x) for x in q]) + '\t \t' +\
    # " ".join([str(x) for x in dq]) + '\t \t' +\
    # " ".join([str(x) for x in ddq]) + '\t \t' +\
    # " ".join([str(x) for x in q_des]) + '\t \t' +\
    # " ".join([str(x) for x in dq_des]) + '\t \t' +\
    # " ".join([str(x) for x in ddq_des]) + '\t \t' +\
    # " ".join([str(x) for x in accel_test]) + '\t \t' +\
    # " ".join([str(x) for x in avel_test]) + '\t \t' +\
    # " ".join([str(x) for x in aaccel_test]) + '\t \t' +\
    # " ".join([str(x) for x in accel_sim]) + '\t \t' +\
    # " ".join([str(x) for x in avel_sim]) + '\t \t' +\
    # " ".join([str(x) for x in aaccel_sim]) + '\t \t' +\
    # " ".join([str(x) for x in g_local_sim]) + '\t \t' +\
    # '\n'

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
    '\n'
    # " ".join([str(x)  for x in pos_ee]) + '\t' +\
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
