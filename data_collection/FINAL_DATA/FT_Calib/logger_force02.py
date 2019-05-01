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
#folder = 'debug/'
# folder = 'hardware/cables_loose'
# folder = 'hardware/no_cables'

folder = 'hardware/fixed_cables/'

#folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "data_file"
# name = "comp_kin_kp100_fc10"

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')


#what it \t for????
#file.write('timestamp\tvelocity based observer\tmomentum based observer\tcontact compensation torques\t' +\
#	'command torques\tdesired position\tcurrent position\n')
#file.write ('timestamp\tcommand torques\tdesired position\tcurrent position\n')
file.write (' tau_com\t ft_sensor \t q \t dq \t qdes\n')
# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)


JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
JOINT_ANGLES_DES_KEY  = "sai2::FrankaPanda::Clyde:::controller::qdes";



# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    tau_com = json.loads(r_server.get(JOINT_TORQUES_COMMANDED_KEY).decode("utf-8"))
    ft_sensor = json.loads(r_server.get(EE_FORCE_SENSOR_FORCE_KEY ).decode("utf-8"))

    q = json.loads(r_server.get(JOINT_ANGLES_KEY).decode("utf-8"))
    dq = json.loads(r_server.get(JOINT_VELOCITIES_KEY).decode("utf-8"))

    q_des = json.loads(r_server.get(JOINT_ANGLES_DES_KEY).decode("utf-8"))


    line = " ".join([str(x) for x in tau_com]) + '\t' +\
    " ".join([str(x) for x in ft_sensor]) + '\t' +\
    " ".join([str(x) for x in q]) + '\t' +\
    " ".join([str(x) for x in dq]) + '\t' +\
    " ".join([str(x) for x in q_des]) + '\t' +\
    '\n'

    file.write(line)

    counter = counter + 1

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
