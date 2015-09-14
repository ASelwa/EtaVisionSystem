# -*- coding: utf-8 -*-
"""
Created on Sun Sep 13 14:34:15 2015

@author: ASelwa
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def filter(data, wn):
    num, denom = signal.butter(8, wn, 'low')
    return signal.filtfilt(num, denom, data, padlen=0)
    
wn = 0.05

def moving_averageV3(x, window):
    """Moving average of 'x' with window size 'window'."""
    return np.convolve(x, np.ones(window)/window, 'same')


# %%

import scipy.signal as sps

filename = "FinalTwo.CSV"
data = np.genfromtxt(filename, delimiter=",", skip_header=1)

# Interpolate time since lost precision (assume constant sampling frequency)
time = np.linspace(data[0, 0], data[-1, 0], len(data[:,0]))  
O2 = data[:, 1]
CO2 = data[:, 2]
caliper = data[:, 3]
disk = data[:, 4]
ambient = data[:, 5]

mainLog = "HallAccValidate8.CSV"
#mainLog = "SLG381.CSV"
dataMain = np.genfromtxt(mainLog, delimiter=",", skip_header=2)

# To align the temp and air data with the speed data
offset = -120 # seconds
plotIndex = 4300 # How many datapoints to plot 

rawTime = dataMain[:, 1] 
rawTime = (rawTime - rawTime[0])/1000 + offset
speed = dataMain[:, 4]/3.6

hallspeed = dataMain[:, 5]/3.6
accelForce = dataMain[:, 10]
propAcc = dataMain[:, 14]

disp = dataMain[:,2]


acc = np.diff(speed)/np.diff(rawTime)
hallacc = np.diff(hallspeed)/np.diff(rawTime)

force = acc*105
hallforce = hallacc*105

filtForce = sps.savgol_filter(force, 13, 1)
filtHallForce = sps.savgol_filter(hallforce, 3, 1)
filtAccelForce = sps.savgol_filter(accelForce, 13, 1)

power = filtForce*(speed[0:-1]+speed[1:])/2
hallpower = filtHallForce*(hallspeed[0:-1]+hallspeed[1:])/2

plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Sun Sept 13")

ax2.plot(rawTime, speed, label='Speed', color='g')
ax2.set_ylabel("Speed (m/s)",fontsize=12)

ax.plot((rawTime[:-1]+rawTime[1:])/2, filtForce, label='Force', color='b')
ax.plot((rawTime[:-1]+rawTime[1:])/2, hallforce, label='Force', color='m')
ax.plot(rawTime, accelForce, label='Force', color='r')
#ax.plot(rawTime, propAcc, label='Force', color='y')
ax.set_ylabel("Force [N]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')


plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Sun Sept 13")

ax2.plot(rawTime, speed, label='Speed', color='g')
ax2.set_ylabel("Speed (m/s)",fontsize=12)


# %%
# CO2 PPM and Speed [m/s]
plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Sun Sept 13")

ax2.plot(rawTime, speed, label='Speed', color='g')
ax2.set_ylabel("Speed (m/s)",fontsize=12)

ax.plot(time, moving_averageV3(CO2, 50), label='CO2', color='b')
ax.set_ylabel("ppm",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')




# %%
# Temperature and Speed
plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Sun Sept 13")

ax2.plot(rawTime, speed, label='Speed', color='b')
ax2.set_ylabel("Speed (m/s)",fontsize=12)

ax.plot(time, caliper, label='Caliper', color='g')
ax.plot(time, disk, label='Disk', color='r')
#ax.plot(time, ambient, label='Ambient', color='y')
ax.set_ylabel("Temperature [C]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')
