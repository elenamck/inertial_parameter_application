#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
#from matplotlib2tikz import save as tikz_save
import numpy as np
from scipy import signal, fftpack

#from scipy.signal import savgol_filter
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

##Just to test
plt.close()

file = np.loadtxt(sys.argv[1] ,skiprows=1)
#print(file)
time = file[1::,0]
q = file[1::,1:5]
q_hat = file[1::,5:9]
avel = file[1::,9:12]
avel_hat = file[1::,12:15]
aaccel = file[1::,15:18]
aaccel_hat = file[1::,18:21]

# ###########Lowpass filter for angular velocity###########
# # Number of samplepoints
# N = avel[:,0].shape[0]
# # sample spacing
# T = 1.0 / 500.0
# x = np.linspace(0.0, N*T, N)
# y_fft = avel[:,0] - np.mean(avel[:,0])
# yf = fftpack.fft(y_fft)
# xf = np.linspace(0.0, 1.0/(2.0*T), N/2)

# fig, ax = plt.subplots()
# ax.plot(xf, 2.0/N * np.abs(yf[:N//2]))
# plt.grid()
# #plt.show()

def butter_lowpass(highcut, sampling_frequency, order):
	nyq = 0.5 * sampling_frequency
	high = highcut / nyq
	b, a = signal.butter(order, high)
	return b,a

def butter_lowpass_direct(order, wn):
	b, a = signal.butter(order, wn)
	return b,a

def butter_lowpass_direct_lfilt(order, wn, data):
	b, a = butter_lowpass_direct(order, wn)
	y = signal.lfilter(b,a,data)
	return y

def butter_lowpass_direct_filtfilt(order, wn, data):
	b, a = butter_lowpass(order, wn)
	y = signal.filtfilt(b,a,data)
	return y

def butter_lowpass_lfilt(data, highcut, sampling_frequency, order):
	b, a = butter_lowpass(highcut, sampling_frequency, order = order)
	y = signal.lfilter(b,a,data)
	return y

def butter_lowpass_filtfilt(data, highcut, sampling_frequency, order):
	b, a = butter_lowpass(highcut, sampling_frequency, order = order)
	y = signal.filtfilt(b,a,data)
	return y


imp_ff_x = butter_lowpass_filtfilt(avel[:,0],17,500,4)
imp_ff_y = butter_lowpass_filtfilt(avel[:,1],17,500,4)
imp_ff_z = butter_lowpass_filtfilt(avel[:,2],17,500,4)


# sig = avel[:,0] - np.mean(avel[:,0])
# print(np.mean(sig))
# sig_fft = fftpack.fft(sig)
# power = np.abs(sig_fft)
# print(power[0])
# sample_freq = fftpack.fftfreq(sig.size, d= 1/500)
# plt.figure()
# plt.plot(sample_freq, power)
# plt.xlabel("Frequency in Hz")
# plt.ylabel("Power")
# plt.title("FFT angular velocity")
# # find peak frequency
# pos_mask = np.where(sample_freq>0)
# freqs = sample_freq[pos_mask]
# peak_freq = freqs[power[pos_mask].argmax()]
# axes = plt.axes([0.55, 0.3, 0.3, 0.5])
# plt.title('Peak frequency')
# plt.plot(freqs[:8], power[:8])
# plt.setp(axes, yticks=[])




aaccel_sg = np.empty_like(avel)
print ("shape sg {}".format(np.shape(aaccel_sg)))
aaccel_sg[:,0] = signal.savgol_filter(avel[:,0],9,6,1)
print ("shape sg_x {}".format(np.shape(aaccel_sg[:,0])))
aaccel_sg[:,1] = signal.savgol_filter(avel[:,1],9,6,1)
print ("shape sg_y {}".format(np.shape(aaccel_sg[:,1])))
aaccel_sg[:,2] = signal.savgol_filter(avel[:,2],9,6,1)
print ("shape sg_z {}".format(np.shape(aaccel_sg[:,2])))

print ("shape time {}".format(np.shape(time)))

for i in np.arange(3):
	print ("shape accel_hat {}".format(np.shape(aaccel_hat[:,i])))

#for i in np.arange(3):
#	aaccel_sg[:,i] = savgol_filter(avel[:,i],19,6,1)

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
turquoise = (0, 0.4667, 0.5412) 


def PlotQuats(time, q_1, q_1_hat, q_2, q_2_hat,q_3, q_3_hat,q_4, q_4_hat):
	fig = plt.figure()
	st= fig.suptitle("Quaternions")
	plt.subplot(4,1,1)
	#plt.tight_layout()
	plt.plot(time, q_1, c=blue, label = "Kinematics")
	plt.plot(time, q_1_hat, c=red, label = "Estimate")
	plt.legend()
	plt.subplot(4,1,2)
	#plt.tight_layout()
	plt.plot(time, q_2, c=blue, label = "Kinematics")
	plt.plot(time, q_2_hat, c=red, label = "Estimate")
	plt.legend()
	plt.subplot(4,1,3)
	#plt.tight_layout()
	plt.plot(time, q_3, c=blue, label = "Kinematics")
	plt.plot(time, q_3_hat, c=red, label = "Estimate")
	plt.legend()
	plt.subplot(4,1,4)
	#plt.tight_layout()
	plt.plot(time, q_4, c=blue, label = "Kinematics")
	plt.plot(time, q_4_hat, c=red, label = "Estimate")
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig("quaternion.png")
	

def PlotThree(time, x_1, x_1_hat, x_2, x_2_hat, x_3, x_3_hat, title_1, title_2, title):
	fig = plt.figure()
	st= fig.suptitle(title)
	plt.title(title)
	plt.subplot(3,1,1)
	plt.plot(time, x_1, c=blue, label = title_1)
	plt.plot(time, x_1_hat, c=red, label = title_2)
	plt.legend()
	plt.subplot(3,1,2)
	plt.plot(time, x_2, c=blue, label = title_1)
	plt.plot(time, x_2_hat, c=red, label = title_2)
	plt.legend()
	plt.subplot(3,1,3)
	plt.plot(time, x_3, c=blue, label = title_1)
	plt.plot(time, x_3_hat, c=red, label = title_2)
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig(title)


def PlotThreeEstimate(time, x_1, x_2, x_3,  title_1, title):
	fig = plt.figure()
	st= fig.suptitle(title)
	plt.title(title)
	plt.subplot(3,1,1)
	plt.plot(time, x_1, c=blue, label = title_1)
	plt.legend()
	plt.subplot(3,1,2)
	plt.plot(time, x_2, c=blue, label = title_1)
	plt.legend()
	plt.subplot(3,1,3)
	plt.plot(time, x_3, c=blue, label = title_1)
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig(title)

def PlotThreeAn(time, x_1_hat,  x_2_hat,  x_3_hat, title):
	fig = plt.figure()
	st= fig.suptitle(title)
	plt.title(title)
	plt.subplot(3,1,1)
	plt.plot(time, x_1_hat, c=red, label = "Estimate")
	plt.legend()
	plt.subplot(3,1,2)
	plt.plot(time, x_2_hat, c=red, label = "Estimate")
	plt.legend()
	plt.subplot(3,1,3)
	plt.plot(time, x_3_hat, c=red, label = "Estimate")
	plt.legend()
	plt.xlim(xmin=0)
	fig.tight_layout()
	st.set_y(0.95)
	fig.subplots_adjust(top=0.85)
	fig.savefig("angularacceleration.png")

# PlotThree(time, avel[:,0], imp_ff_x, avel[:,1], imp_ff_y, avel[:,2], imp_ff_z, "measurements", "forward-backward filter", "Butterworth Lowpass forward-backward")
# PlotQuats(time, q[:,0], q_hat[:,0], q[:,1], q_hat[:,1], q[:,2], q_hat[:,2], q[:,3], q_hat[:,3])
#PlotThree(time, avel_hat[:,0], avel[:,0], avel_hat[:,1], avel[:,1], avel_hat[:,2], avel[:,2], "Estimate", "Sensor", "Angular Velocity")
PlotThree(time, avel[:,0], avel_hat[:,0], avel[:,1], avel_hat[:,1], avel[:,2], avel_hat[:,2],"Sensor", "Estimate", "Angular Velocity")

# PlotThreeEstimate(time, avel_hat[:,0], avel_hat[:,1], avel_hat[:,2], "Estimate", "Angular Velocity")
# PlotThreeEstimate(time, aaccel[:,0], aaccel[:,1], aaccel[:,2], "Kinematics", "Angular Acceleration")
# PlotThree(time, aaccel_hat[:,0],aaccel_sg[:,0], aaccel_hat[:,1], aaccel_sg[:,1],  aaccel_hat[:,2],aaccel_sg[:,2],"Estimate","SG Filter","Angular Acceleration")
PlotThreeEstimate(time, aaccel_hat[:,0], aaccel_hat[:,1], aaccel_hat[:,2], "Estimate", "Angular Acceleration")

plt.show()

