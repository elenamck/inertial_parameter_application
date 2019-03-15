#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy import signal, fftpack
# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

##Just to test


file = np.loadtxt(sys.argv[1] ,skiprows=1)
#print(file)
pos = file[1::,0:3]
vel = file[1::,3:6]
acc = file[1::,6:9]
kf_pos = file[1::,9:12]
kf_vel = file[1::,12:15]
kf_acc = file[1::,15:18]
g_local = file[1::,18:21]

time = np.arange(np.size(pos[:,0]))
time = time/1000


#TUM colors rgb
blue = (0,0.3961,0.7412)
red = (0.7686,0.0275,0.1059)
green =(0,0.4863,0.1882)
orange =  (0.8902, 0.4471, 0.1333)
purple = (0.4118, 0.0314, 0.3529)
grey = (0.6118, 0.6157, 0.6235)
yellow = (0.9765, 0.7294, 0)
#turquoise = (0, 0.4667, 0.5412) 

colors = blue, red, green, orange, purple, grey, yellow




def PlotNoLimOne(time, F_x1, F_y1, F_z1, label_name_x1,label_name_y1, label_name_z1, axis_label_x, axis_label_y):
	plt.figure()
	plt.plot(time, F_x1, c=blue, label = label_name_x1)
	plt.plot(time, F_y1, c=red, label = label_name_y1)
	plt.plot(time, F_z1, c=green, label = label_name_z1)
	plt.xlabel(axis_label_x)
	plt.ylabel(axis_label_y)
	plt.xlim(xmin=0)
	plt.legend()


def PlotCompare(counter, time, x_1, x_2, y_1, y_2, z_1, z_2, title_1, title_2, title_3, label_1, label_2,title, savename):
	plt.figure(counter, figsize=(7,8))
	plt.title(title)
	plt.subplot(311)
	plt.tight_layout()
	plt.plot(time, x_1, c=blue, label= label_1)
	plt.plot(time, x_2, c=red, label= label_2)
	plt.title(title_1)
	plt.legend()
	# plt.xlabel("Elapsed time in s")
	plt.subplot(312)
	plt.tight_layout()
	plt.plot(time, y_1, c=blue, label= label_1)
	plt.plot(time, y_2, c=red, label= label_2)
	plt.title(title_2)
	plt.legend()
	# plt.xlabel("Elapsed time in s")
	plt.subplot(313)
	plt.tight_layout()
	plt.plot(time, z_1, c=blue, label= label_1)
	plt.plot(time, z_2, c=red, label= label_2)
	plt.title(title_3)
	plt.legend()
	plt.savefig(savename)





# PlotNoLimOne(time, pos[:,0], pos[:,1], pos[:,2], "pos_x", "pos_y", "pos_z", "Elapsed time in s", "Position", "position.png")
# PlotNoLimOne(time, vel[:,0], vel[:,1], vel[:,2], "v_x", "v_y", "v_z", "Elapsed time in s", "Velocity", "velocity.png")
# PlotNoLimOne(time, acc[:,0], acc[:,1], acc[:,2], "acc_x", "acc_y", "acc_z", "Elapsed time in s", "Acceleration", "acceleration.png")
# PlotNoLimOne(time, kf_pos[:,0], kf_pos[:,1], kf_pos[:,2], "kf_pos_x", "kf_pos_y", "kf_pos_z", "Elapsed time in s", "KF Position", "kf_position.png")
# PlotNoLimOne(time, kf_vel[:,0], kf_vel[:,1], kf_vel[:,2], "kf_v_x", "kf_v_y", "kf_v_z", "Elapsed time in s", "KF Velocity", "kf_velocity.png")
# PlotNoLimOne(time, kf_acc[:,0], kf_acc[:,1], kf_acc[:,2], "kf_acc_x", "kf_acc_y", "kf_acc_z", "Elapsed time in s", "KF Acceleration", "kf_acceleration.png")



PlotCompare(1, time, pos[:,0], kf_pos[:,0], pos[:,1], kf_pos[:,1],pos[:,2], kf_pos[:,2],"Position in x","Position in y","Position in z", "Kinematics", "Kalman Filter","Position Simulation" ,"position.png")
PlotCompare(2, time, vel[:,0], kf_vel[:,0], vel[:,1], kf_vel[:,1],vel[:,2], kf_vel[:,2],"Velocity in x","Velocity in y","Velocity in z", "Kinematics", "Kalman Filter", "Velocity Simulation", "velocity.png")
PlotCompare(3, time, acc[:,0], kf_acc[:,0], acc[:,1], kf_acc[:,1],acc[:,2], kf_acc[:,2],"Acceleration in x","Acceleration in y","Acceleration in z", "Sensor", "Kalman Filter", "Acceleration Simulation", "acceleration.png")
PlotNoLimOne(time, g_local[:,0], g_local[:,1], g_local[:,2], "Gravity in x", "Gravity in y", "Gravity in z", "Elapsed time in s", "Gravity in m/s^2" )
sig = acc[:,0] - np.mean(acc[:,0])
print(np.mean(sig))
sig_fft = fftpack.fft(sig)
power = np.abs(sig_fft)
print(power[0])
sample_freq = fftpack.fftfreq(sig.size, d= 1/500)
plt.figure()
plt.plot(sample_freq, power)
plt.xlabel("Frequency in Hz")
plt.ylabel("Power")
plt.title("FFT linear acceleration")

plt.show()

