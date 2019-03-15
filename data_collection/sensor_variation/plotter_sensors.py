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
accel = file[1::,0:3]
avel = file[1::,3:6]
q = file[1::,6:13]


time = np.arange(np.size(accel[:,0]))
time = time/100


#TUM colors rgb
blue = (0,0.3961,0.7412)
red = (0.7686,0.0275,0.1059)
green =(0,0.4863,0.1882)
orange =  (0.8902, 0.4471, 0.1333)
purple = (0.4118, 0.0314, 0.3529)
grey = (0.6118, 0.6157, 0.6235)
yellow = (0.9765, 0.7294, 0)
turquoise = (0, 0.4667, 0.5412) 

# var_accel_x = np.var(accel[:,0])
# var_accel_y = np.var(accel[:,1])
# var_accel_z = np.var(accel[:,2])
var_accel = np.array([np.var(accel[:,0]), np.var(accel[:,1]), np.var(accel[:,2])])
print("variance in linear acceleration: %s" % var_accel)

# var_avel_x = np.var(avel[:,0])
# var_avel_y = np.var(avel[:,1])
# var_avel_z = np.var(avel[:,2])
var_avel = np.array([np.var(avel[:,0]),np.var(avel[:,1]),np.var(avel[:,2])])
print("variance in angular velocity: %s" % var_avel)

# var_q_1 = np.var(q[:,0])
# var_q_2 = np.var(q[:,1])
# var_q_3 = np.var(q[:,2])
# var_q_4 = np.var(q[:,3])
# var_q_5 = np.var(q[:,4])
# var_q_6 = np.var(q[:,5])
# var_q_7 = np.var(q[:,6])

var_q = np.array([np.var(q[:,0]),np.var(q[:,1]),np.var(q[:,2]),np.var(q[:,3]),np.var(q[:,4]),np.var(q[:,5]),np.var(q[:,6])])
print("variance in joint angles: %s" % var_q)


colors = blue, red, green, orange, purple, grey, yellow

def PlotThree(time, x_1, x_2 , x_3, title, savename):
	fig = plt.figure()
	st= fig.suptitle(title)
	plt.title(title)
	plt.subplot(3,1,1)
	plt.plot(time, x_1, c=blue, label = "x")
	plt.legend()
	plt.subplot(3,1,2)
	plt.plot(time, x_2, c=red, label = "y")
	plt.legend()
	plt.subplot(3,1,3)
	plt.plot(time, x_3, c=green, label = "z")
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig("savename")

def PlotAngles(time, x_1,  x_2, x_3, x_4, x_5, x_6, x_7):
	fig = plt.figure(figsize=(10,10))
	st= fig.suptitle("Joint angles")
	plt.subplot(7,1,1)
	plt.plot(time, x_1, c=blue, label = "q_1")
	plt.legend()
	plt.subplot(7,1,2)
	plt.plot(time, x_2, c=red, label = "q_2")
	plt.legend()
	plt.subplot(7,1,3)
	plt.plot(time, x_3, c=green, label = "q_3")
	plt.legend()
	plt.subplot(7,1,4)
	plt.plot(time, x_4, c=orange, label = "q_4")
	plt.legend()
	plt.subplot(7,1,5)
	plt.plot(time, x_5, c=purple, label = "q_5")
	plt.legend()
	plt.subplot(7,1,6)
	plt.plot(time, x_6, c=grey, label = "q_6")
	plt.legend()
	plt.subplot(7,1,7)
	plt.plot(time, x_7, c=grey, label = "q_7")
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig("encoder_noise.png")






PlotThree(time, accel[:,0], accel[:,1], accel[:,2], "linear acceleration in g", "accelerometer_noise.png")
PlotThree(time, avel[:,0], avel[:,1], avel[:,2], "angular velocity", "gyroscope_noise.png")
PlotAngles(time, q[:,0],q[:,1],q[:,2],q[:,3],q[:,4],q[:,5],q[:,6])

plt.show()
