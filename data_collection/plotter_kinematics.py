#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

##Just to test


file = np.loadtxt(sys.argv[1] ,skiprows=1)
#print(file)
a_vel = file[1::,0:3]
a_vel_kin = file[1::,3:6]
l_acc = file[1::,6:9]
l_acc_kin = file[1::,9:12]
time = np.arange(np.size(a_vel[:,0]))
time = time/100


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


# def PlotNoLim(time, F_x1, F_y1, F_z1, label_name_x1,label_name_y1, label_name_z1,F_x2, F_y2, F_z2, label_name_x2,label_name_y2, label_name_z2, axis_label_x, title, savename):
# 	plt.figure()
# 	plt.subplot(2,1,1)
# 	plt.plot(time, F_x1, c=blue, label = label_name_x1)
# 	plt.plot(time, F_y1, c=red, label = label_name_y1)
# 	plt.plot(time, F_z1, c=green, label = label_name_z1)
# 	plt.subplot(2,1,2)
# 	plt.plot(time, F_x2, c=blue, label = label_name_x2)
# 	plt.plot(time, F_y2, c=red, label = label_name_y2)
# 	plt.plot(time, F_z2, c=green, label = label_name_z2)
# 	plt.xlabel(axis_label_x)
# 	plt.xlim(xmin=0)
# 	plt.title(title)
# 	plt.legend()
# 	plt.savefig(savename)

# def PlotNoLimOne(time, F_x1, F_y1, F_z1, label_name_x1,label_name_y1, label_name_z1, axis_label_x, axis_label_y, savename):
# 	plt.figure()
# 	plt.plot(time, F_x1, c=blue, label = label_name_x1)
# 	plt.plot(time, F_y1, c=red, label = label_name_y1)
# 	plt.plot(time, F_z1, c=green, label = label_name_z1)
# 	plt.xlabel(axis_label_x)
# 	plt.ylabel(axis_label_y)
# #	plt.xlim(xmin=0)
# 	plt.legend()
# #	plt.savefig(savename)

def PlotCompare(counter, time, x_1, x_2, y_1, y_2, z_1, z_2, title_1, title_2, title_3, label_1, label_2):
	plt.figure(counter, figsize=(7,8))
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
	# plt.xlabel("Elapsed time in s")


PlotCompare(1, time, a_vel[:,0], a_vel_kin[:,0], a_vel[:,1], a_vel_kin[:,1],a_vel[:,2], a_vel_kin[:,2],"Angular Velocity in x","Angular Velocity in y","Angular Velocity in z", "Sensor", "Kinematics")
PlotCompare(2, time, l_acc[:,0], l_acc_kin[:,0], l_acc[:,1], l_acc_kin[:,1],l_acc[:,2], l_acc_kin[:,2],"Linear Acceleration in x","Linear Acceleration in y","Linear Acceleration in z", "Sensor", "Kinematics")


# PlotNoLimOne(time, a_vel[:,0], a_vel[:,1], a_vel[:,2], "omega_x", "omega_y", "omega_z", "Elapsed time in s", "Angular Velocity in m/s", "angular_velocity_sim.png")
# PlotNoLimOne(time, l_acc[:,0], l_acc[:,1], l_acc[:,2], "a_x", "a_y", "a_z", "Elapsed time in s", "Linear Acceleration in m/s^2", "linear_acceleration_sim.png")
# PlotNoLimOne(time, a_acc[:,0], a_acc[:,1], a_acc[:,2], "alpha_x", "alpha_y", "alpha_z", "Elapsed time in s", "Angular Acceleration in m/s^2", "linear_acceleration_sim.png")



plt.show()

