#!/usr/bin/env python3
# coding: utf-8

# In[31]:


import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import filtfilt, butter
#%matplotlib inline


# # Function definition 
# 

# ## Butterworth filter
# - digital filter, infinite impulse response
# - Butterworth: has maximally flat frequency response in the passband 
# - fc: cutoff frequency (Hz)
# - fs: sampling frequency (Hz)
# - applied using filtfilt:
#     - signal twice, once forwards and once backwards
#     - order of resulting filter is twice the original filter order
# 

# In[32]:


def filter_butter(input_vector, fcutt, fsampling, order):
    # nyquist = fs/2
    # wc = fc / nyquist
    output_vector = np.zeros(input_vector.shape)
    for i in np.arange(3):
        xn = input_vector[:,i]
        b, a = butter(order,fcutt, fs = fsampling)
        if not np.all(np.abs(np.roots(a)) < 1):
            print('Filter with cutoff at {} Hz is unstable given ' 'sample frequency {} Hz'.format(cutoff, fs))
        y = filtfilt(b, a, xn)
        output_vector[:,i] = y
    return output_vector


# ## Derivatives of measurements

# - Computes derivatives f_dot = f(t+dt) - f(t)/dt

# In[33]:


def get_derivative(measurement, logger_frequency):
    derivatives = np.zeros(measurement.shape)
    derivatives[1:,:]=measurement[1:]-measurement[:-1]
    derivatives[0]=derivatives[1]
    derivatives = derivatives * logger_frequency
    return derivatives


# - Computes derivatives f_dot = f(t+dt) - f(t)/dt then filters signal with butterworth filter

# In[34]:


def ComputeDotFiltered(input_vector,cutoff_filter,logger_freq, order_filter):
        input_vector_dot = get_derivative(input_vector,logger_freq)
        output_vector = filter_butter(input_vector,cutoff_filter,logger_freq, order_filter)
        return output_vector
    
def ComputeFilteredDotFiltered(input_vector,cutoff_filter,logger_freq, order_filter):
        input_vector_filtered = filter_butter(input_vector,cutoff_filter,logger_freq, order_filter)
        output_vector = ComputeDotFiltered(input_vector_filtered ,cutoff_filter,  logger_freq, order_filter)
        return output_vector


# ## Fourier series in Python

# In[35]:


def fourier_series_q(a, b, axis, N, w_f, q0, time):
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
        q[i] += q0[i]
        for j in np.arange(N):
            q[i] += M_a[j,i] * np.sin(arg*(j+1)) + M_b[j,i] * np.cos(arg*(j+1))
    return q

def fourier_series_dq(a, b, axis, N, w_f, q0, time):
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
            dq[i] += M_a[j,i] * arg_diff * (j+1) * np.cos(arg*(j+1)) - M_b[j,i] * arg_diff * (j+1) * np.sin(arg*(j+1))      
    return dq

def fourier_series_ddq(a, b, axis, N, w_f, q0, time):
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
        
            ddq[i] += - M_a[j,i] * arg_diff_diff * (j+1)*(j+1) * np.sin(arg*(j+1)) - M_b[j,i] * arg_diff_diff * (j+1)*(j+1) * np.cos(arg*(j+1))
            
    return ddq


# ## Fouriertransform

# In[36]:


def fouriertransform(data, frequency):
    Fs = frequency
    Ts = 1.0 / Fs
    n = len(data[:,0])
    k = np.arange(n)
    T = n/Fs
    frq_two_sides = k/T
    frq_one_side = frq_two_sides[range(int(n/2.0))]
    Y = np.fft.fft(data)/n
    Y = Y[range(int(n/2.0))]
    


# ## Rotation from IMU frame to Force Sensor frame

# In[37]:


def rotate_in_force_sensor_frame(data_in_IMU_frame):
    data_in_FS_frame = np.zeros(data_in_IMU_frame.shape)
    
    data_in_FS_frame[:,0] = data_in_IMU_frame[:,2]
    data_in_FS_frame[:,1] = data_in_IMU_frame[:,0]
    data_in_FS_frame[:,2] = data_in_IMU_frame[:,1]
    
    return data_in_FS_frame


# In[38]:


def principal_moments(phi):
    Theta = np.zeros([3,3])
    theta = np.zeros(phi[:,0:3].shape)
    for i, number in enumerate(phi[:,0]):
        Theta = np.array([[phi[i,4], phi[i,5],phi[i,6]],[phi[i,5],phi[i,7],phi[i,8]],[phi[i,6],phi[i,8],phi[i,9]]])
        diag = np.diag(Theta)
        theta[i, :] = diag
    return theta


# ## Objects

# ### Kinematics Object

# In[39]:



class AngularKinematics:
    def __init__(self, avel, aaccel):
        self.avel = avel
        self.aaccel = aaccel
    def assign_label(self, label):
        self.label = label
    def assign_file(self, file):
        self.file = file
class ExtendedKalmanFilter(AngularKinematics):
    def __init__(self, ori, avel, aaccel):
        AngularKinematics.__init__(self, avel, aaccel)
        self.ori = ori
    def addInputEKF(self, ori_input, avel_input):
        self.ori_in = ori_input
        self.avel_in = avel_input
    def addAaccel(self, aaccel_ref):
        self.aaccel_ref = aaccel_ref


class Kinematics(AngularKinematics):
    def __init__(self, accel, avel, aaccel):
        AngularKinematics.__init__(self, avel, aaccel)
        self.accel = accel
    def computeDerivatives(self, freq):
        self.accel_dot = get_derivative(self.accel,freq)
        self.avel_dot = get_derivative(self.avel,freq)
        self.aaccel_dot = get_derivative(self.aaccel,freq)
    def computeFilteredDerivatives(self,fc, freq, order):    
        computeDerivatives(freq)
        self.filt_accel_dot_filt = ComputeFilteredDotFiltered(self.accel,fc, freq, order)
        self.filt_avel_dot_filt = ComputeFilteredDotFiltered(self.avel,fc, freq, order)
        self.filt_aaccel_dot_filt = ComputeFilteredDotFiltered(self.aaccel,fc,freq, order)
        
        self.accel_dot_filt = ComputeDotFiltered(self.accel,fc, freq, order)
        self.avel_dot_filt = ComputeDotFiltered(self.avel,fc, freq, order)
        self.aaccel_dot_filt = ComputeDotFiltered(self.aaccel,fc, freq, order)
        
class KinematicsData(Kinematics):
    def __init__(self, pos, vel, accel, avel, aaccel):
        Kinematics.__init__(self, accel, avel, aaccel)
        self.pos = pos
        self.vel = vel
    def addGravity(self,g_local):
        self.gravity = g_local
    def addOrientation(self, orientation):
        self.ori = orientation
    def computeDerivatives(self, freq):
        Kinematics.computeDerivatives(self)
        self.pos_dot =  get_derivative(self.pos,freq)
        self.vel_dot = get_derivative(self.vel,freq)
        
    def computeFilteredDerivatives(self, freq, fc, order):
        Kinematics.computeFilteredDerivatives(self, freq, fc, order)
        
        self.filt_pos_dot_filt = ComputeFilteredDotFiltered(self.pos, fc, freq, order)
        self.filt_vel_dot_filt = ComputeFilteredDotFiltered(self.vel, fc, freq, order)
        
        self.pos_dot_filt = ComputeDotFiltered(self.pos,fc, freq, order)
        self.vel_dot_filt = ComputeDotFiltered(self.vel,fc, freq, order)
        
class Input(Kinematics):
    def __init__(self, accel, avel, aaccel, g_local):
        Kinematics.__init__(self, accel, avel, aaccel)
        self.gravity = g_local
  


        

class FTSensor:
    def __init__(self, force, torque):
        self.force = force
        self.torque = torque
    def fuse_ft_data(self):
        self.force_torque =  np.concatenate((self.force, self.torque), axis = 1)
    def assign_label(self, label):
        self.label = label
        
class IMUSensor: 
    def __init__(self,accel,avel):
        self.accel = accel
        self.avel = avel
        self.accel_ft_frame = np.zeros(self.accel.shape)
        self.avel_ft_frame = np.zeros(self.avel.shape)
    def assign_label(self, label):
        self.label = label

    def rotate_data(self):
        self.accel_ft_frame = rotate_in_force_sensor_frame(self.accel)
        self.avel_ft_frame = rotate_in_force_sensor_frame(self.avel)
   
    def rotate_and_correct_data(self, g_local):
        self.rotate_data()
        self.accel_ft_frame_corrected_aux = 9.81 * self.accel_ft_frame
        self.accel_ft_frame_corrected = g_local + self.accel_ft_frame_corrected_aux


class EstimationInput(Input, FTSensor):
    def __init__(self, accel, avel, aaccel, g_local, force, torque):
        Input.__init__(self, accel, avel, aaccel, g_local)
        FTSensor.__init__(self, force, torque)

class EstimationAlgorithm():
    def __init__(self,phi):
        self.phi = phi        
    def addAlgorithmSecond(self, phi_2):
        self.phi_2 = phi_2

    def computeMassSecond(self):
        self.mass_2 = self.phi_2[:,0]
    def computeCOMSecond(self):
        self.com_2 = np.zeros((len(self.mass_2), 3))
        for i in np.arange(3):
            if 0 in self.mass_2[:]: 
                self.com_2[:,i] = 0
            if not 0 in self.mass_2[:]:
                self.com_2[:,i] =  self.phi_2[:,1+i]/self.mass_2[:]
class EstimationAlgorithmInfo(EstimationAlgorithm):
    def __init__(self,phi):
        EstimationAlgorithm.__init__(self,phi)
    def addAlgorithmInfoFirst(self, filter_size, lambda_coeff):
        self.filter_size = filter_size
        self.lambda_coeff = lambda_coeff
    def addAlgorithmInfoSecond(self, filter_size, lambda_coeff):
        self.filter_size_2 = filter_size
        self.lambda_coeff_2 = lambda_coeff

class Estimation(EstimationInput, EstimationAlgorithmInfo):
    def __init__(self, accel, avel, aaccel, g_local, force, torque, phi):
        EstimationInput.__init__(self, accel, avel, aaccel, g_local, force, torque)
        EstimationAlgorithmInfo.__init__(self,phi)

class EstimationTwo(Estimation):
    def __init__(self, accel, avel, aaccel, g_local, force, torque, phi, phi_2):
        Estimation.__init__(self, accel, avel, aaccel, g_local, force, torque, phi)
        self.mass = self.phi[:,0]
        self.com = self.phi[:,1:4]
        if 0 in self.phi[:,0]: 
            self.com[:,0] = 0
            self.com[:,1] = 0
            self.com[:,2] = 0
        else:
            self.com[:,0] /= self.phi[:,0]
            self.com[:,1] /= self.phi[:,0]
            self.com[:,2] /= self.phi[:,0]
        self.phi_2 = phi_2
        self.mass_2 = self.phi_2[:,0]
        self.com_2 = self.phi_2[:,1:4]
        if 0 in self.phi_2[:,0]: 
            self.com_2[:,0] = 0
            self.com_2[:,1] = 0
            self.com_2[:,2] = 0
        else:
            self.com_2[:,0] /= self.phi_2[:,0]
            self.com_2[:,1] /= self.phi_2[:,0]
            self.com_2[:,2] /= self.phi_2[:,0]

    def assignPrincipalMoments(self, principal_moments, principal_moments_2):
        self.principal_moments = principal_moments
        self.principal_moments_2 = principal_moments_2



class Sensors (IMUSensor, FTSensor):
    def __init__(self,accel,avel,force, torque):
        IMUSensor.__init__(self,accel,avel)
        FTSensor.__init__(self, force, torque)
    

        
        

            
        
        


# ### Joint Trajectory Objects

# In[40]:


class JointData:
    
    def __init__(self, q, dq):
        self.q = q
        self.dq = dq
    def assign_label(self, label):
        self.label = label
        
class JointTrajectory(JointData):
    def __init__(self, q, q_des,dq, dq_des):
        JointData.__init__(self, q, dq)
        self.q_des = q_des
        self.dq_des = dq_des
    def axis(self, axis):
        self.axis = axis
        self.q_axis = self.q[:,7-axis::]
        self.dq_axis = self.dq[:,7-axis::]
    
    def fourier(self, time, a, b, N, fourier_frequency, sampling_frequency, initial_config):    
        if initial_config.shape != self.axis:
            self.q0 = initial_config[7-self.axis::]
        else:
            self.q0 = initial_config 
        self.N = N
        self.a = a
        self.b = b
        self.t = time
        self.q0 *= np.pi/180.0
        self.Ts = 1/sampling_frequency
        self.w_f = fourier_frequency
        
        self.q_fourier = np.array([fourier_series_q(self.a,self.b,self.axis,self.N, self.w_f, self.q0, t) for t in self.t])
        self.dq_fourier = np.array([fourier_series_dq(self.a,self.b,self.axis,self.N, self.w_f, self.q0, t) for t in self.t])
        

        
class JointTrajectorySim(JointTrajectory):
    def __init__(self, q, q_des, dq, dq_des, ddq, ddq_des):
        JointTrajectory.__init__(self, q, q_des, dq, dq_des)
        self.ddq = ddq
        self.ddq_des = ddq_des
        
    def axis(self, axis):
        JointTrajectory.axis(self,axis)
        self.ddq_axis = self.ddq[:,7-axis::]
        
    def fourier(self, time, a, b, N, fourier_frequency, sampling_frequency, initial_config):
        JointTrajectory.fourier(self, time, a, b, N, fourier_frequency, sampling_frequency, initial_config)
        self.ddq_fourier = np.array([fourier_series_ddq(self.a,self.b,self.axis,self.N, self.w_f, self.q0, t) for t in self.t])



# ### Controller Gain Tuning Objects

# In[41]:


class Gains:
    def __init__(self, k_vec):
        self.k = k_vec
class PIDController(Gains):
    def __init__(self, k_vec):
        Gains.__init__(self, k_vec)
        
        # self.kp = self.k[0]
        # self.kv = self.k[1]
        # self.ki = self.k[2]

    def assign_label(self, label):
        self.label = label
    def assign_file(self, file):
        self.file = file


class JointController(JointTrajectory, PIDController):
    def __init__(self, q, q_des,dq, dq_des, k_vec):
        JointTrajectory.__init__(self, q, q_des,dq, dq_des)
        PIDController.__init__(self,k_vec)

        
class PosController(PIDController):
    def __init__(self, pos_des, pos_curr, vel_des, vel_curr, k_vec_pos):
        PIDController.__init__(self,k_vec_pos)
        self.pos_des = pos_des
        self.pos_curr = pos_curr
        self.vel_des = vel_des
        self.vel_curr = vel_curr       

class OriController(PIDController):
    def __init__(self, ori_des, ori_curr, avel_des, avel_curr, k_vec_ang):
        PIDController.__init__(self,k_vec_ang)       
        self.ori_des = ori_des
        self.ori_curr =ori_curr
        self.avel_des = avel_des
        self.avel_curr = avel_curr       

class PosOriController(PosController, OriController):
    def __init__(self, pos_des, pos_curr, vel_des, vel_curr, k_vec_pos,ori_des, ori_curr, avel_des, avel_curr, k_vec_ang ):
        PosController.__init__(self, pos_des, pos_curr, vel_des, vel_curr, k_vec_pos)
        OriController.__init__(self, ori_des, ori_curr, avel_des, avel_curr, k_vec_ang)
    


# ## Plotting Functions
# ### TUM Colors

# In[42]:


blue = (0,0.3961,0.7412)
red = (0.7686,0.0275,0.1059)
green =(0,0.4863,0.1882)
orange =  (0.8902, 0.4471, 0.1333)
purple = (0.4118, 0.0314, 0.3529)
grey = (0.6118, 0.6157, 0.6235)
yellow = (0.9765, 0.7294, 0)
color_list = [blue, red, green, orange, grey, purple, yellow]


# ### Common labels
# #### Object class

# In[43]:


class Label:
    def __init__(self, name, abbreviation, unit):
        self.name = name;
        self.abbreviation = abbreviation;
        self.unit = unit


# In[44]:


pos_info = Label("Position","$r$", "$m$")
vel_info = Label("Linear  velocity","$v$", r"$\frac{m}{s}$")
accel_info = Label("Linear acceleration", "$a$", r"$\frac{m}{s^2}$")

avel_info = Label("Angular velocity", r"$\omega$", r"$\frac{rad}{s}$")
aaccel_info = Label("Angular acceleration", r"$\alpha$", r"$\frac{rad}{s^2}$")

force_info = Label("Force", r"$F$", r"$N$")
torque_info = Label("Torque", r"$\tau$", r"$Nm$")

frequency_info = Label("Frequency", "$f$" , r"$Hz$")
mass_info = Label("Mass", "$m$", "$kg$")
com_info = Label("Center of mass", "$com$", "$m$")
principal_moments_info = Label("Prinicipal moments of inertia", "$I$", "$kg m^2$")

prismatic_joint_info = Label("Prismatic joint displacement",'$p$', r"$m$")
revolute_joint_info = Label("Revolute joint displacement","$q$", "$rad$")
prismatic_joint_vel_info = Label("Prismatic joint velocity",r'$\dot{p}$', r"$\frac{m}{s}$")
revolute_joint_vel_info = Label("Revolute joint displacement",r"$\dot{q}$", r"$\frac{rad}{s}$")

# In[45]:


# class PosOriLabeled(PosoriController, Label):
#     def __init__(self, pos_des, pos_curr, vel_des, vel_curr, k_vec_pos,ori_des, ori_curr, avel_des, avel_curr, k_vec_ang):
        
    


# In[46]:

def DataLoading(datafilename, relative_path):
    data_file = datafilename
    path = relative_path
    path_data_file = path + data_file
    file = np.loadtxt(path_data_file,skiprows=1)

    return file

def ValueAssignment_Trajectory(file, axis, simulation = False):
    if(simulation == False):
        q   = file[0::,0:7]  
        dq  = file[0::,7:14]  
        q_des = file[0::,14:14+axis]
        dq_des = file[0::,14+axis:14+2*axis]
        joint_trajectory_task = JointTrajectory(q, q_des,dq, dq_des) 
        joint_trajectory_task.axis(axis)  
    elif (simulation == True):
        q   = file[0::,0:7]  
        dq  = file[0::,7:14] 
        ddq = file[0::,14:21] 
        q_des = file[0::,21:21+axis]
        dq_des = file[0::,21+axis:21+2*axis]
        ddq_des = file[0::,21+2*axis:21+3*axis]

        joint_trajectory_task = JointTrajectorySim(q, q_des,dq, dq_des, ddq, ddq_des) 
        joint_trajectory_task.axis(axis)  

    return joint_trajectory_task

def ValueAssignment_Sensor(file_sensor, sensor_type, g_local, rotate = True):
    if(sensor_type == 'imu'):
        accel = file_sensor[0::,0:3]
        avel = file_sensor[0::,3:6]
        sensor_task = IMUSensor(accel, avel)
        sensor_task.rotate_and_correct_data(g_local)
    elif(sensor_type == 'ft'):
        force = file_sensor[0::,0:3]
        torque = file_sensor[0::,3:6]
        sensor_task = FTSensor(force, torque)
        sensor_task.fuse_ft_data()
    elif (sensor_type == 'both'):
        accel = file_sensor[0::,0:3]
        avel = file_sensor[0::,3:6]
        force = file_sensor[0::,6:9]
        torque = file_sensor[0::,9:12]
        sensor_task = Sensors(accel, avel, force, torque)
        sensor_task.fuse_ft_data()
        sensor_task.rotate_and_correct_data(g_local)
    return sensor_task


def ValueAssignment_Estimation(file_estimate, g_local, two_estimates=True):
    accel = file_estimate[0::,0:3]
    avel = file_estimate[0::,3:6]
    aaccel = file_estimate[0::,6:9]
    force = file_estimate[0::,9:12]
    torque = file_estimate[0::,12:15]
    phi = file_estimate[0::,15:25]
    gravity = g_local
    if(two_estimates == True):
        phi_2 = file_estimate[0::,25:35]
        estimation_task = EstimationTwo(accel, avel, aaccel, gravity, force, torque, phi, phi_2)
    elif(two_estimates == False):
        estimation_task = EstimationTwo(accel, avel, aaccel, gravity, force, torque, phi)
    return estimation_task

def ValueAssignmant_EKF_InOut(file_ekf):
    if (len(file_ekf[0,:]) == 21):
        quat_in = file_ekf[:,1:5]
        quat_est = file_ekf[:,5:9]
        avel_in = file_ekf[:,9:12]
        avel_est = file_ekf[:,12:15]
        aaccel_in = file_ekf[:,15:18]
        aaccel_est = file_ekf[:,18:21]
        ekf_task = ExtendedKalmanFilter(quat_est, avel_est, aaccel_est)
        ekf_task.addInputEKF(quat_in, avel_in)
        ekf_task.addAaccel(aaccel_in)
        return ekf_task



def DataLoading_ValueAssignment_EstimationTuning(datafilename, relative_path, f_c_lp, d_lp, fc_butter):
    file = DataLoading(datafilename, relative_path)
    print('The length of the given file is: {}'.format(len(file[0,:])))
    g_local = file[0::, 82:85]
    file_low_pass_estimate = file[0::,0:35]
    estimation_task_lowpass = ValueAssignment_Estimation(file_low_pass_estimate, g_local, two_estimates = True)
    estimation_task_lowpass.assign_label('cutt-off frequency: ' + str(f_c_lp) + ' damping: ' + str(d_lp))
    file_butter_estimate = file[0::,35:70]
    estimation_task_butter = ValueAssignment_Estimation(file_butter_estimate, g_local, two_estimates = True)
    estimation_task_butter.assign_label('cutt-off frequency: ' + str(fc_butter))
    file_sensors = file[0::,70:82]
    sensors_task = ValueAssignment_Sensor(file_sensors, 'both', g_local)
    sensors_task.assign_label(datafilename)
    return estimation_task_lowpass, estimation_task_butter, sensors_task

def DataLoading_ValueAssignment_Estimation(datafilename, relative_path, axis = 0):
    file = DataLoading(datafilename, relative_path)
    print('The length of the given file is: {}'.format(len(file[0,:])))
    accel   = file[0::,0:3]    
    avel   = file[0::,3:6]  
    aaccel  = file[0::,6:9]     
    gravity  = file[0::,9:12] 
    force = file[0::,12:15]
    torque  = file[0::,15:18]
    if (len(file[0,:]) == 18):
        print ('output object: EstimationInput')
        estimation_input_task = EstimationInput(accel, avel, aaccel, gravity, force, torque)
        estimation_input_task.fuse_ft_data()
        estimation_input_task.assign_label(datafilename)
    if (len(file[0,:])> 18):
        phi = file[0::,18:28]
        estimation_task = Estimation(accel, avel, aaccel, gravity, force, torque, phi)
        estimation_task.fuse_ft_data()
        estimation_task.assign_label(datafilename)
        if (len(file[0,:]) == 28):
            print ('output object: Estimation')
            return estimation_task
        elif (axis != 0 and len(file[0,:]) == 49 + 3*axis):
            print ('Output objects: Estimation and JointTrajectorySim')
            file_trajectory = file[0::, 28:49 + 3*axis]
            joint_trajectory_task = ValueAssignment_Trajectory(file_trajectory, axis, simulation = True)
            joint_trajectory_task.assign_label(datafilename)
            return estimation_task, joint_trajectory_task
        elif (axis != 0 and len(file[0,:]) != 49 + 3*axis):
            file_trajectory = file[0::, 28:42 + 2*axis]
            joint_trajectory_task = ValueAssignment_Trajectory(file_trajectory, axis, simulation = False)
            joint_trajectory_task.assign_label(datafilename)
            if (axis != 0 and len(file[0,:]) == 42 + 2*axis):
                print('Output objects: Estimation and JointTrajectory') 
                return estimation_task, joint_trajectory_task
            elif (axis != 0 and len(file[0,:]) == 48 + 2*axis):
                print('Output objects: Estimation and JointTrajectory, IMUSensor')
                file_imu = file[0::,42+2*axis : 48+2*axis]
                imu_sensor_task = ValueAssignment_Sensor(file_imu, 'imu', gravity)
                imu_sensor_task.assign_label(datafilename)
                return estimation_task, joint_trajectory_task, imu_sensor_task
            elif (axis != 0 and len(file[0,:]) == 54 + 2*axis):
                print('Output objects: Estimation and JointTrajectory, Sensors')
                file_sensors = file[0::,42+2*axis : 54+2*axis]
                sensors_task = ValueAssignment_Sensor(file_sensors, 'both', gravity)
                sensors_task.assign_label(datafilename)
                return estimation_task, joint_trajectory_task, sensors_task


def DataLoading_ValueAssignmentCartesianTrajectory(datafilename, relative_path):
    data_file = datafilename
    path = relative_path
    path_data_file = path + data_file

    file = np.loadtxt(path_data_file,skiprows=2)
    pos_des   = file[0::,0:3]    
    pos_curr   = file[0::,3:6]  
    vel_des  = file[0::,6:9]     
    vel_curr  = file[0::,9:12] 
    if (len(file[0,:]))> 12:
        ori_des = file[0::,12:16]
        ori_curr  = file[0::,16:20]
        avel_des = file[0::,20:23]
        avel_curr = file[0::,23:26]
        posori_task = PosOriController(pos_des, pos_curr, vel_des, vel_curr,0, ori_des, ori_curr, avel_des,avel_curr ,0)
        posori_task.assign_file(datafilename)
        return posori_task
    if (len(file[0,:])) == 12:
        pos_task = PosController(pos_des, pos_curr,0 , vel_des, vel_curr,0)
        pos_task.assign_file(datafilename)
        return pos_task

def DataLoading_ValueAssignment_FTSensor(datafilename, relative_path):
    file_ft = DataLoading(datafilename, relative_path)
    print('The length of the given file is: {}'.format(len(file_ft[0,:])))
    data_ft = file_ft[0::, 7:13]
    ft_task = ValueAssignment_Sensor(data_ft, 'ft' , np.zeros(3), False)
    ft_task.assign_label(datafilename)
    return ft_task      

def DataLoading_ValueAssignment_EKF(datafilename, relative_path):
    file_ekf = DataLoading(datafilename, relative_path)
    ekf_task = ValueAssignmant_EKF_InOut(file_ekf)
    ekf_task.assign_file(datafilename)
    return ekf_task

def DataLoading_ValueAssignment_AllStates(datafilename, relative_path):
    file = DataLoading(datafilename, relative_path)
    ekf_task = ValueAssignmant_EKF_InOut(file_ekf)
    ekf_task.assign_file(datafilename)
    return ekf_task

# #### x-label

# In[47]:


x_label = "Time $t$ in $s$"
x_label_ms = "Time $t$ in $ms$"

coordinates = ["$_x$","$_y$","$_z$"]


# #### y-label for joint trajectoy

# In[48]:


#labels for comparing actual desired

joint_angles = ['$q_1$',r'$q_2$',r'$q_3$',r'$q_4$',r'$q_5$',r'$q_6$',r'$q_7$']
joint_angles_des = [r'$q_{1_{\mathrm{des}}}$',r'$q_{2_{\mathrm{des}}}$',r'$q_{3_{\mathrm{des}}}$',r'$q_{4_{\mathrm{des}}}$',r'$q_{5_{\mathrm{des}}}$',r'$q_{6_{\mathrm{des}}}$', r'$q_{7_{\mathrm{des}}}$']
y_label_joint_angles = "Joint angles $q$ in $rad$"


joint_velocities = [r'$\dot{q}_1$',r'$\dot{q}_2$',r'$\dot{q}_3$',r'$\dot{q}_4$',r'$\dot{q}_5$',r'$\dot{q}_6$',r'$\dot{q}_7$']
joint_velocities_des = ['$\dot{q}_{1_{\mathrm{des}}}$',r'$\dot{q}_{2_{\mathrm{des}}}$',r'$\dot{q}_{3_{\mathrm{des}}}$',r'$\dot{q}_{4_{\mathrm{des}}}$',r'$\dot{q}_{5_{\mathrm{des}}}$',r'$\dot{q}_{6_{\mathrm{des}}}$',r'$\dot{q}_{7_{\mathrm{des}}}$']
y_label_joint_velocities = r"Joint velocities $\dot{q}$ in $\frac{rad}{s}$" 

joint_accelerations = ['$\ddot{q}_1$',r'$\ddot{q}_2$',r'$\ddot{q}_3$',r'$\ddot{q}_4$',r'$\ddot{q}_5$',r'$\ddot{q}_6$',r'$\ddot{q}_7$']
joint_accelerations_des = ['$\ddot{q}_{1_{\mathrm{des}}}$',r'$\ddot{q}_{2_{\mathrm{des}}}$',r'$\ddot{q}_{3_{\mathrm{des}}}$',r'$\ddot{q}_{4_{\mathrm{des}}}$',r'$\ddot{q}_{5_{\mathrm{des}}}$',r'$\ddot{q}_{6_{\mathrm{des}}}$',r'$\ddot{q}_{7_{\mathrm{des}}}$']
y_label_joint_accelerations = r"Joint accelerations $\ddot{q}$ in $\frac{rad}{s^2}$"    


#labels for comparing simulation and hardware
joint_angles_simulation = [r'$q_{1_{\mathrm{simulation}}}$',r'$q_{2_{\mathrm{simulation}}}$',r'$q_{3_{\mathrm{simulation}}}$',r'$q_{4_{\mathrm{simulation}}}$',r'$q_{5_{\mathrm{simulation}}}$',r'$q_{6_{\mathrm{simulation}}}$', r'$q_{7_{\mathrm{simulation}}}$']
joint_velocities_simulation = ['$\dot{q}_{1_{\mathrm{simulation}}}$',r'$\dot{q}_{2_{\mathrm{simulation}}}$',r'$\dot{q}_{3_{\mathrm{simulation}}}$',r'$\dot{q}_{4_{\mathrm{simulation}}}$',r'$\dot{q}_{5_{\mathrm{simulation}}}$',r'$\dot{q}_{6_{\mathrm{simulation}}}$',r'$\dot{q}_{7_{\mathrm{simulation}}}$']
joint_accelerations_simulation = ['$\ddot{q}_{1_{\mathrm{simulation}}}$',r'$\ddot{q}_{2_{\mathrm{simulation}}}$',r'$\ddot{q}_{3_{\mathrm{simulation}}}$',r'$\ddot{q}_{4_{\mathrm{simulation}}}$',r'$\ddot{q}_{5_{\mathrm{simulation}}}$',r'$\ddot{q}_{6_{\mathrm{simulation}}}$',r'$\ddot{q}_{7_{\mathrm{simulation}}}$']

joint_angles_hardware = [r'$q_{1_{\mathrm{hardware}}}$',r'$q_{2_{\mathrm{hardware}}}$',r'$q_{3_{\mathrm{hardware}}}$',r'$q_{4_{\mathrm{hardware}}}$',r'$q_{5_{\mathrm{hardware}}}$',r'$q_{6_{\mathrm{hardware}}}$', r'$q_{7_{\mathrm{hardware}}}$']
joint_velocities_hardware = ['$\dot{q}_{1_{\mathrm{hardware}}}$',r'$\dot{q}_{2_{\mathrm{hardware}}}$',r'$\dot{q}_{3_{\mathrm{hardware}}}$',r'$\dot{q}_{4_{\mathrm{hardware}}}$',r'$\dot{q}_{5_{\mathrm{hardware}}}$',r'$\dot{q}_{6_{\mathrm{hardware}}}$',r'$\dot{q}_{7_{\mathrm{hardware}}}$']
# joint_accelerations_hardware = ['$\ddot{q}_{1_{\mathrm{hardware}}}$',r'$\ddot{q}_{2_{\mathrm{hardware}}}$',r'$\ddot{q}_{3_{\mathrm{hardware}}}$',r'$\ddot{q}_{4_{\mathrm{hardware}}}$',r'$\ddot{q}_{5_{\mathrm{hardware}}}$',r'$\ddot{q}_{6_{\mathrm{hardware}}}$',r'$\ddot{q}_{7_{\mathrm{hardware}}}$']


# ### Plotting function definitions

# #### Align y axis when x shared

# In[49]:


def align_yaxis(ax1, ax2):
    y_lims = np.array([ax.get_ylim() for ax in [ax1, ax2]])

    # force 0 to appear on both axes, comment if don't need
    y_lims[:, 0] = y_lims[:, 0].clip(None, 0)
    y_lims[:, 1] = y_lims[:, 1].clip(0, None)

    # normalize both axes
    y_mags = (y_lims[:,1] - y_lims[:,0]).reshape(len(y_lims),1)
    y_lims_normalized = y_lims / y_mags

    # find combined range
    y_new_lims_normalized = np.array([np.min(y_lims_normalized), np.max(y_lims_normalized)])

    # denormalize combined range to get new axes
    new_lim1, new_lim2 = y_new_lims_normalized * y_mags
    ax1.set_ylim(new_lim1)
    ax2.set_ylim(new_lim2)


# #### Fouriertransfrom

# In[50]:


def plot_fourier_transform(data, frequency, x_lim_min, x_lim_max):
    
    Fs = frequency
    Ts = 1.0 / Fs
    n = len(data)
    k = np.arange(n)
    T = n/Fs
    frq_two_sides = k/T
    frq_one_side = frq_two_sides[range(int(n/2.0))]
    Y = np.fft.fft(data)/n
    Y = Y[range(int(n/2.0))]
    
    plt.figure(figsize=(6,7))

    plt.plot(frq_one_side,abs(Y),c=blue,linewidth=2.0 ) # plotting the spectrum
    plt.grid(True)
    plt.xlabel(frequency_info.name + " "  + frequency_info.abbreviation + " in " + frequency_info.unit)
    plt.xlim([x_lim_min, x_lim_max])
    plt.ylabel("Power Spectral Density")
    


   


# In[51]:


def plot_fourier_transform_xyz(data, frequency, x_lim_min, x_lim_max,y_lim_max ):
    coordinates = ["$_x$","$_y$","$_z$"]
    Fs = frequency
    Ts = 1.0 / Fs
    n = len(data[:,0])
    k = np.arange(n)
    T = n/Fs
    f, axarr = plt.subplots(1,3,figsize=(15,5))
    for idx,ax in enumerate(axarr):
        frq_two_sides = k/T
        frq_one_side = frq_two_sides[range(int(n/2.0))]
        Y = np.fft.fft(data[:,idx])/n
        Y = Y[range(int(n/2.0))]
        ax.plot(frq_one_side,abs(Y), c=color_list[idx], linewidth=2.0, label = frequency_info.abbreviation + str(coordinates[idx])) 
        ax.set_xlabel(frequency_info.name + " "  + frequency_info.abbreviation + " in " + frequency_info.unit)
        ax.set_ylabel("Power Spectral Density")
        ax.set_xlim([x_lim_min, x_lim_max])
        ax.set_ylim([0,y_lim_max])
        ax.grid(True)
        ax.legend(loc=0)
    f.tight_layout()


# #### Joint trajectories

# In[52]:
def joint_data(angles, derivative):
    plt.figure(figsize=(9,7))
    if derivative == 0:
        joint_lables= joint_angles
        y_label = y_label_joint_angles
    if derivative == 1:
        joint_lables= joint_velocities
        y_label = y_label_joint_velocities
    if derivative == 2:
        joint_lables= joint_accelerations
        y_label = y_label_joint_accelerations
    for i in np.arange(np.size(angles[0,:])):
        plt.plot(angles[:, i], c = color_list[i], label = joint_lables[i], linewidth=2.0 )
        plt.xlabel(x_label_ms)
        plt.ylabel(y_label)
        plt.xlim(0, np.size(angles[:,0]))
        plt.grid(True)
        plt.legend()

def joint_angles_compare(axis, angles, angles_des, derivative):
    plt.figure(figsize=(9,7))
    if derivative == 0:
        joint_lables= joint_angles
        joint_lables_des= joint_angles_des
        y_label = y_label_joint_angles
    if derivative == 1:
        joint_lables= joint_velocities
        joint_lables_des= joint_velocities_des
        y_label = y_label_joint_velocities
    if derivative == 2:
        joint_lables= joint_accelerations
        joint_lables_des= joint_accelerations_des
        y_label = y_label_joint_accelerations
    joint_lables = joint_lables[7-axis::]
    joint_lables_des = joint_lables_des[7-axis::]
    for i, lable_q, in enumerate(joint_lables):
        plt.plot(angles[:, i], c = color_list[i], label = lable_q, linewidth=2.0 )
        plt.plot(angles_des[:, i], c=color_list[i], label = joint_lables_des[i] , linewidth=2.0, linestyle = '--')
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xlim(left = 0)
        plt.grid(True)
        plt.legend()
        
def joint_angles_sim_hard(axis, angles_sim, angles_hardware, derivative):
    if derivative == 0:
        joint_lables_sim = joint_angles_simulation
        joint_lables_hw = joint_angles_hardware
        y_label = y_label_joint_angles
    if derivative == 1:
        joint_lables_sim = joint_velocities_simulation
        joint_lables_hw = joint_velocities_hardware
        y_label = y_label_joint_velocities    
    joint_lables_sim = joint_lables_sim[7-axis::]
    joint_lables_hw = joint_lables_hw[7-axis::]
    f, axarr = plt.subplots(1,2,figsize=(15,9))
    for i, lable_sim, in enumerate(joint_lables_sim):
        axarr[0].plot(angles_sim[:, i], c = color_list[i], label = lable_sim, linewidth=2.0 )
        axarr[0].set_xlim(left = 0)
        axarr[0].set_ylabel(y_label)
        axarr[0].grid(True)
        axarr[0].legend()
    for i, lable_hw in enumerate(joint_lables_hw):
        axarr[1].plot(angles_hardware[:, i], c = color_list[i], label = lable_hw, linewidth=2.0 )
        axarr[1].set_xlim(left = 0)
        axarr[1].set_ylabel(y_label)
        axarr[1].grid(True)
        axarr[1].legend()
    axarr[0].set_xlabel(x_label)
    axarr[1].set_xlabel(x_label)


# #### Three dimensional variables

# In[53]:


def xyz(data, variable_name, variable_abbreviation, variable_unit):
    plt.figure(figsize=(9,7))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx, coordinate in enumerate(coordinates):
        plt.plot(data[:,idx], c = color_list[idx], label = variable_abbreviation + str(coordinate),  linewidth=2.0)
        plt.xlabel(x_label)
        plt.ylabel(variable_name + " " + variable_abbreviation + " in " + variable_unit)
        plt.xlim(left = 0)
        plt.grid(True)
        plt.legend()
        
def xyz_obj(data, object_info):
    plt.figure(figsize=(9,7))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx, coordinate in enumerate(coordinates):
        plt.plot(data[:,idx], c = color_list[idx], label = object_info.abbreviation + str(coordinate),  linewidth=2.0)
        plt.xlabel(x_label)
        plt.ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        plt.xlim(left = 0)
        plt.grid(True)
        plt.legend()
def xyz_obj_notime(data, object_info):
    plt.figure(figsize=(9,7))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx, coordinate in enumerate(coordinates):
        plt.plot(data[:,idx], c = color_list[idx], label = object_info.abbreviation + str(coordinate),  linewidth=2.0)
        plt.xlabel(x_label_ms)
        plt.ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        plt.xlim(0, np.size(data[:,0]))
        plt.grid(True)
        plt.legend()

def xyz_subplots_three_inputs_two_axes(data_1, data_2, data_3, object_info1,object_info2,label_list):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        ax2 = ax.twinx()
        ax2.plot(data_3[:,idx], c=color_list[2], linewidth=2.0, label = object_info2.abbreviation + str(coordinates[idx]) + " " + label_list[1])
        ax.plot(data_1[:,idx], c=color_list[0], linewidth=2.0, label = object_info1.abbreviation + str(coordinates[idx]) )
        ax.set_xlim( left = 0)
        ax.set_ylabel(object_info1.name + " " + object_info1.abbreviation + " in " + object_info1.unit)
        ax.grid(True)
        ax.set_xlabel(x_label_ms)
        ax2.plot(data_2[:,idx], c=color_list[1], linewidth=2.0, label = object_info2.abbreviation + str(coordinates[idx]) + " " + label_list[0]) 
        ax2.set_ylabel(object_info2.name + " " + object_info2.abbreviation + " in " + object_info2.unit)
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc=0)
        align_yaxis(ax, ax2)
    f.tight_layout()
    
def xyz_subplots_two_inputs_two_axes(data_1, data_2, object_info1,object_info2 ,label_list):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        ax2 = ax.twinx()
        ax.plot(data_1[:,idx], c=color_list[0], linewidth=2.0, label = object_info1.abbreviation + str(coordinates[idx])+ " " + label_list[0])
        ax.set_xlim(left = 0)
        ax.set_ylabel(object_info1.name + " " + object_info1.abbreviation + " in " + object_info1.unit)
        ax.grid(True)
        ax.set_xlabel(x_label_ms)
        ax2.plot(data_2[:,idx], c=color_list[1], linewidth=2.0, label = object_info2.abbreviation + str(coordinates[idx]) + " " + label_list[1]) 
        ax2.set_ylabel(object_info2.name + " " + object_info2.abbreviation + " in " + object_info2.unit)
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc=0)
        align_yaxis(ax, ax2)
    f.tight_layout()
    
def xyz_subplots_two_inputs(data_1, data_2, object_info,label_list):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        ax.plot(data_1[:,idx], c=color_list[0], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[0])
        ax.set_xlim(left = 0)
        ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        ax.set_xlabel(x_label)
        ax.grid(True)
        ax.plot(data_2[:,idx], c=color_list[1], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[1]) 
        lines, labels = ax.get_legend_handles_labels()
        ax.legend(lines , labels , loc=0)
    f.tight_layout()
    
def xyz_subplots_three_inputs(data_1, data_2,data_3, object_info,label_list):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        ax.plot(data_1[:,idx], c=color_list[0], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[0])
        ax.plot(data_2[:,idx], c=color_list[1], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[1]) 
        ax.plot(data_3[:,idx], c=color_list[2], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[2]) 
        ax.set_xlim(left = 0)
        ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        ax.set_xlabel(x_label)
        lines, labels = ax.get_legend_handles_labels()
        ax.grid(True)
        ax.legend(lines , labels , loc=0)
    f.tight_layout()
    
# def xyz_subplots_four_inputs(data_1, data_2,data_3, data_4, object_info,label_list):
def xyz_subplots_four_inputs(data_1, data_2,data_3, data_4, object_info,label_list):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        ax.plot( data_1[:,idx], c=color_list[0], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[0])
        ax.plot( data_2[:,idx], c=color_list[1], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[1]) 
        ax.plot( data_3[:,idx], c=color_list[2], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[2]) 
        ax.plot( data_4[:,idx], c=color_list[3], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[3]) 
#         ax.set_xlim(left = 0)
        ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        ax.set_xlabel(x_label)
        lines, labels = ax.get_legend_handles_labels()
        ax.grid(True)
        ax.legend(lines , labels , loc=0)
    f.tight_layout()
    


# #### Estimated inertial parameters

# In[54]:


def Plot_intertial_params_one_error(data,  real_values):
    f, axarr = plt.subplots(10,1,figsize=(6,15))
    params = ['Mass in $kg$', 'COM_x in $m$','COM_y in $m$', 'COM_z in $m$',r'$I_{xx}$',r'$I_{xy}$',r'$I_{xz}$',r'$I_{yy}$',r'$I_{yz}$',r'$I_{zz}$']
    if 0 in data[:,0]: 
        data[:,1] = 0
        data[:,2] = 0
        data[:,3] = 0
    else:
        data[:,1] /= data[:,0]
        data[:,2] /= data[:,0]
        data[:,3] /= data[:,0]
    mse = np.empty_like(data)
    for idx,value in enumerate(real_values):                           #for all 10 inertial params  
        for i in np.arange(np.size(data[:,0])):                       #elementwise 
            mse[i,idx] = (np.square(data[i,idx] - value)).mean(axis=None)
    for idx, param in enumerate(params):
        axarr[idx].plot(mse[:,idx], c=blue)
        axarr[idx].set_title(param)
        axarr[idx].set_xlim(left = 0)
    axarr[9].set_xlabel(x_label)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.tight_layout()
    
def intertial_params_one(data, real_values):
    f, axarr = plt.subplots(4,1,figsize=(6,15))
    params = ['Mass in $kg$', 'COM_x in $m$','COM_y in $m$', 'COM_z in $m$',r'$I_{xx}$',r'$I_{xy}$',r'$I_{xz}$',r'$I_{yy}$',r'$I_{yz}$',r'$I_{zz}$']
    if 0 in data[:,0]: 
        data[:,1] = 0
        data[:,2] = 0
        data[:,3] = 0
    else:
        data[:,1] /= data[:,0]
        data[:,2] /= data[:,0]
        data[:,3] /= data[:,0]
    mass = data[:,0]
    com = data[:,1:4]
    com_label = ["$c_x$","$c_y$","$c_z$"]
    com_real = np.array([real_values[1],real_values[2],real_values[3]])
    princip = principal_moments(data) 
    princip_real = np.array([real_values[4],real_values[7],real_values[9]])
    princip_label = ["$I_{xx}$","$I_{yy}$","$I_{zz}$"]
    for idx in np.arange(3):
        axarr[idx].set_xlim(left = x_lim_min)
        axarr[idx].axhline(real_values[idx], c=red)
    axarr[0].plot(mass, c = blue)
    axarr[0].set_ylabel(mass_info.name + " " + mass_info.abbreviation + " in " + mass_info.unit)
    axarr[0].axhline(real_values[0], c = blue , linestyle = "--")
    for i in np.arange(3):
        axarr[1].plot(com[:,i], c = color_list[i], label = com_info.abbreviation + str(coordinates[i]),linewidth=2.0)
        # axarr[1].axhline(com_real[i], c=color_list[i], linestyle = "--")
        axarr[2].plot(princip[:,i], c = color_list[i], label = princip_label[i],linewidth=2.0)
        # axarr[2].axhline(princip_real[i], c=color_list[i], linestyle = "--")
    axarr[1].set_ylabel(com_info.name + " " + com_info.abbreviation + " in " + com_info.unit)
    axarr[2].set_ylabel(principal_moments_info.name + " " + principal_moments_info.abbreviation + " in " + principal_moments_info.unit)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)


# #### Comparing Simulation and hardware

# In[55]:


def sim_hard(data_sim, data_hardware,variable_name, variable_abbreviation, variable_unit,label_1, label_2 , plot_quantity):
    if plot_quantity ==1:
        f, axarr = plt.subplots(1,2,figsize=(15,9))
        coordinates = ["$_x$","$_y$","$_z$"]
        for idx,coordinate in enumerate(coordinates):
            axarr[0].plot(data_sim[:,idx], c=color_list[idx], linewidth=2.0, label = variable_abbreviation + str(coordinate) + " " + label_1)
            axarr[0].set_xlim(left = 0)
            axarr[0].set_ylabel(variable_name + " " + variable_abbreviation + " in " + variable_unit)
            axarr[0].grid(True)
            axarr[0].legend()
        for idx,coordinate in enumerate(coordinates):
            axarr[1].plot(data_hardware[:,idx], c=color_list[idx], linewidth=2.0, label = variable_abbreviation + str(coordinate) + " " + label_2)
            axarr[1].set_xlim(left = 0)
            axarr[1].set_ylabel(variable_name + " " + variable_abbreviation + " in " + variable_unit)
            axarr[1].grid(True)
            axarr[1].legend()
        axarr[0].set_xlabel(x_label)
        axarr[1].set_xlabel(x_label)
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        
    if plot_quantity == 3:
        f, axarr = plt.subplots(3,2,figsize=(15,9))
        coordinates = ["$_x$","$_y$","$_z$"]
        for idx,coordinate in enumerate(coordinates):
            axarr[idx,0].plot(data_sim[:,idx], c=color_list[idx], linewidth=2.0, label = variable_abbreviation + str(coordinate) + " " + label_1)
            axarr[idx,0].set_xlim(left = 0)
            axarr[idx,0].set_ylabel(variable_name + " " + variable_abbreviation + " in " + variable_unit)
            axarr[idx,0].grid(True)
            axarr[idx,0].legend()
        for idx,coordinate in enumerate(coordinates):
            axarr[idx,1].plot(data_hardware[:,idx], c=color_list[idx], linewidth=2.0, label = variable_abbreviation + str(coordinate) + " " + label_2)
            axarr[idx,1].set_xlim(left = 0)
            axarr[idx,1].set_ylabel(variable_name + " " + variable_abbreviation + " in " + variable_unit)
            axarr[idx,1].grid(True)
            axarr[idx,1].legend()
        axarr[2,0].set_xlabel(x_label)
        axarr[2,1].set_xlabel(x_label)
        plt.setp([a.get_xticklabels() for a in axarr[0, :]], visible=False)
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)


# #### Force and Torque

# In[56]:


def force_torque(data_1, data_2):
    f, axarr = plt.subplots(3,2,figsize=(15,9))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,coordinate in enumerate(coordinates):
        axarr[idx,0].plot(data_1[:,idx], c=color_list[idx], linewidth=2.0)
        axarr[idx,0].set_xlim(left = 0)
        axarr[idx,0].set_ylabel("$F$"+str(coordinate)+" in " "$N$")
        axarr[idx,0].grid(True)
    for idx,coordinate in enumerate(coordinates):
        axarr[idx,1].plot(data_2[:,idx], c=color_list[idx], linewidth=2.0)
        axarr[idx,1].set_xlim(left = 0)
        axarr[idx,1].set_ylabel(r"$\tau$"+str(coordinate)+" in " "$Nm$")
        axarr[idx,1].grid(True)
    axarr[2,0].set_xlabel(x_label)
    axarr[2,1].set_xlabel(x_label)
    plt.setp([a.get_xticklabels() for a in axarr[0, :]], visible=False)
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)


# #### Gain Tuning

# In[57]:


def plot(info, *args):
    f, axarr = plt.subplots(3,1,figsize=(15,15))        
    for ax in axarr:
        for idx, data in enumerate(*args):
            ax.plot(data[:,idx], c=color_list[0], linewidth=2.0, label = "desired")
            ax.plot(data[:,idx], c=color_list[1], linewidth=2.0, label = "current, kp = "+ controller.kp_pos +", kv = "+ controller.kv_pos + " ki = "+ controller.ki_pos) 
            ax.set_xlim(left = 0)
            ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
            ax.set_xlabel(x_label)
            lines, labels = ax.get_legend_handles_labels()
            ax.grid(True)
            ax.legend(lines , labels , loc=0)
    f.tight_layout()


# In[58]:


def posori_plot(var_type, *args):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    data_label = ["desired", "current, kp = "+ controller.kp_pos +", kv = "+ controller.kv_pos + " ki = "+ controller.ki_pos]
    if var_type == "pos":
        
        for ax in axarr:
            for idx, controller in enumerate(*args):
                ax.plot(controller.pos_des[:,idx], c=color_list[0], linewidth=2.0, label = data_label[0])
                ax.plot(controller.pos_curr[:,idx], c=color_list[1], linewidth=2.0, label = data_label[1]) 
                ax.set_xlim(left = 0)
                ax.set_ylabel(pos_info.name + " " +pos_info.abbreviation + " in " + pos_info.unit)
                ax.set_xlabel(x_label)
                lines, labels = ax.get_legend_handles_labels()
                ax.grid(True)
                ax.legend(lines , labels , loc=0)
        f.tight_layout()
    if var_type == "vel":

        for ax in axarr:
            for idx, controller in enumerate(*args):
                ax.plot(controller.vel_des[:,idx], c=color_list[0], linewidth=2.0, label = data_label[0])
                ax.plot(controller.vel_curr[:,idx], c=color_list[1], linewidth=2.0, label = data_label[1]) 
                ax.set_xlim(left = 0)
                ax.set_ylabel(pos_info.name + " " +pos_info.abbreviation + " in " + pos_info.unit)
                ax.set_xlabel(x_label)
                lines, labels = ax.get_legend_handles_labels()
                ax.grid(True)
                ax.legend(lines , labels , loc=0)
        f.tight_layout()
    

def xyz_subplots_varinputs(object_info, label_list, *datas):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    color_list_aux = [red, green, blue, yellow, grey]
    for idx,ax in enumerate(axarr):
        for j, data in enumerate(datas):
            ax.plot(data[:,idx], c=color_list_aux[j], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[j])
            ax.set_xlim(left = 0)
            ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
            ax.set_xlabel(x_label_ms)
            lines, labels = ax.get_legend_handles_labels()
            ax.grid(True)
            ax.legend(lines , labels , loc=0)
    f.tight_layout()    

def mass_varinputs(object_info, label_list, *datas):
    plt.figure(figsize=(15,5))
    for j, data in enumerate(datas):
        plt.plot(data, c=color_list[j], linewidth=2.0, label = object_info.abbreviation + " " + label_list[j])
        plt.xlim(left = 0)
        plt.ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        plt.xlabel(x_label_ms)
        plt.grid(True)
        plt.legend()

def xyz_subplots_varinputs_est(object_info, label_list, *datas):
    f, axarr = plt.subplots(3,1,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    for idx,ax in enumerate(axarr):
        i = 0
        for j, data in enumerate(datas):
            if(j %2 == 0):
                ax.plot(data[:,idx], c=color_list[i], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[i] + " estimation")
            elif(j %2 == 1):
                ax.plot(data[:,idx], c=color_list[i], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]) + " " + label_list[i] + " sensor", linestyle = "--")
                i += 1
            ax.set_xlim(left = 0)
            ax.set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
            ax.set_xlabel(x_label_ms)
            lines, labels = ax.get_legend_handles_labels()
            ax.grid(True)
            ax.legend(lines , labels , loc=0)
    f.tight_layout()    

def force_torque_bias(sensor_object, unbiased_object):
    f, axarr = plt.subplots(3,2,figsize=(15,15))
    coordinates = ["$_x$","$_y$","$_z$"]
    data_sensor = sensor_object.force_torque
    data_unbiased = unbiased_object.force_torque
    mean = np.zeros(3)
    for idx,coordinate in enumerate(coordinates):
        object_info = force_info
        diff = sensor_object.force_torque[:,idx] - unbiased_object.force_torque[:,idx]
        mean[idx] = np.mean(diff)
        axarr[idx,0].plot(diff, c=color_list[idx], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]))
        axarr[idx,0].set_xlim(left = 0)
        axarr[idx,0].set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        axarr[idx,0].axhline(mean[idx], c= grey, label = unbiased_object.label + "\n mean " + object_info.abbreviation + str(coordinates[idx]) + " = " + str(mean[idx]), linestyle = "--", linewidth=2.0)
        axarr[idx,0].legend()
        axarr[idx,0].grid(True)
    for idx,coordinate in enumerate(coordinates):
        object_info = torque_info
        diff = sensor_object.force_torque[:,idx+3] - unbiased_object.force_torque[:,idx+3]
        mean[idx] = np.mean(diff)
        axarr[idx,1].plot(diff, c=color_list[idx], linewidth=2.0, label = object_info.abbreviation + str(coordinates[idx]))
        axarr[idx,1].set_xlim(left = 0)
        axarr[idx,1].set_ylabel(object_info.name + " " + object_info.abbreviation + " in " + object_info.unit)
        axarr[idx,1].axhline(mean[idx], c= grey, label =  unbiased_object.label + "\n mean " + object_info.abbreviation + str(coordinates[idx]) + " = " + str(mean[idx]), linestyle = "--", linewidth=2.0)
        axarr[idx,1].legend()
        axarr[idx,1].grid(True)

    axarr[2,0].set_xlabel(x_label_ms)
    axarr[2,1].set_xlabel(x_label_ms)
    plt.setp([a.get_xticklabels() for a in axarr[0, :]], visible=False)
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)