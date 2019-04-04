# -*- coding: utf-8 -*-
#!/usr/bin/env python3

# read text files

import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy import signal, fftpack
from scipy.signal import savgol_filter
from constraint import *

#TUM colors rgb
blue = (0,0.3961,0.7412)
red = (0.7686,0.0275,0.1059)
green =(0,0.4863,0.1882)
orange =  (0.8902, 0.4471, 0.1333)
purple = (0.4118, 0.0314, 0.3529)
grey = (0.6118, 0.6157, 0.6235)
yellow = (0.9765, 0.7294, 0)
color_list = [blue, red, green, orange, grey, purple, yellow]

def fourier_series(a, b, axis, N, w_f, q0, time):
    M_a = np.zeros(shape=(N, axis))
    M_b = np.zeros(shape=(N, axis))
    for i in np.arange(axis):
        for j in np.arange(N):
            M_a[j, i] = a[j+(i*2)]
            M_b[j, i] = b[j+(i*2)]
            
    arg = w_f * time
    arg_diff = w_f
    arg_diff_diff = w_f*w_f

    q = np.zeros(shape=(axis))
    dq = np.zeros(shape=(axis))
    ddq = np.zeros(shape=(axis))
    for i in np.arange(axis):
        q[i] = q0[i]
        for j in np.arange(N):
            q[i] += M_a[j,i] * np.sin(arg*(j+1)) + M_b[j,i] * np.cos(arg*(j+1))
            dq[i] += M_a[j,i] * arg_diff * (j+1) * np.cos(arg*(j+1)) - M_b[j,i] * arg_diff * (j+1) * np.sin(arg*(j+1))
            ddq[i] += - M_a[j,i] * arg_diff_diff * (j+1)*(j+1) * np.sin(arg*(j+1)) - M_b[j,i] * arg_diff_diff * (j+1)*(j+1) * np.cos(arg*(j+1))
            
    return q



problem = Problem()
problem.addVariable('a', range(5))
problem.addVariable('b', range(5))
problem.addConstraint(lambda a, b: a + b == 5)
problem.addConstraint(lambda a, b: a * b == 6)
problem.getSolutions()