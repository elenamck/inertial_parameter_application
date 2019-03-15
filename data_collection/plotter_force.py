#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
from matplotlib2tikz import save as tikz_save
from math import pow, sqrt
import numpy as np
import sys
from numpy import linalg as LA
# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

##Just to test


file = np.loadtxt(sys.argv[1] ,skiprows=1)
#print(file)
time = file[1::,0]
phi = file[1::,1:11]

def error_norm(X,Y): 						#X: parameter in list, Y: real Value
	for i in range(len(phi[:,X])):
		phi[i,X] -=Y
		phi[i,X] = np.power(phi[i,X],2)
		phi[i,X] = np.sqrt(phi[i,X])

error_norm(0,1.5)
error_norm(1,0.6)
error_norm(2,0.6)
error_norm(3,0.6)


time = np.arange(np.shape(time)[0])
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



def PlotNoLimOne(time, F_x1, F_y1, F_z1,F_w1, label_name_x1,label_name_y1, label_name_z1,label_name_w1, axis_label_x, axis_label_y, savename):
	plt.figure()
	plt.plot(time, F_x1, c=blue, label = label_name_x1)
	plt.plot(time, F_y1, c=red, label = label_name_y1)
	plt.plot(time, F_z1, c=green, label = label_name_z1)
	plt.plot(time, F_w1, c=purple, label = label_name_w1)
	plt.xlabel(axis_label_x)
	plt.ylabel(axis_label_y)
	plt.xlim(xmin=0)
	plt.legend()
	plt.savefig(savename)



PlotNoLimOne(time, phi[:,0], phi[:,1], phi[:,2],phi[:,3], "e_{m}", "e_{cx}", "e_{cy}","e_{cz}", "Elapsed time in s", "Error in inertial parameters", "error_inertialparams3.png")





# PlotNoLim(time, accel[:,0],  accel[:,1],  accel[:,2], "linear Acceleration x", "linear Acceleration y", "linear Acceleration z", "elapsed Time in s", "Linear Acceleration") 
# PlotNoLim(time, aaccel[:,0],  aaccel[:,1],  aaccel[:,2], "angular Acceleration x", "angular Acceleration y", "angular Acceleration z", "elapsed Time in s", "Angular Acceleration") 
# PlotNoLim(time, aavel[:,0],  aavel[:,1],  aavel[:,2], "angular Velocity x", "angular Velocity y", "angular Velocity z", "elapsed Time in s", "Linear Velocity") 

# plt.figure()
# plt.plot(time, q[:,0], c=blue)
# plt.plot(time, q[:,1], c=red)
# plt.plot(time, q[:,2], c=green)
# plt.plot(time, q[:,3], c=orange)
# plt.plot(time, q[:,4], c=purple)
# plt.plot(time, q[:,5], c=grey)
# plt.plot(time, q[:,6], c=yellow)
plt.show()

