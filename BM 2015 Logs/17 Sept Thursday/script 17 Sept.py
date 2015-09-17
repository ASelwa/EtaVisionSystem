# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 09:42:57 2015

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

import scipy.signal as sps
# %%



filename = "Morn.CSV"
data = np.genfromtxt(filename, delimiter=",", skip_header=1)

# Interpolate time since lost precision (assume constant sampling frequency)
time = np.linspace(data[0, 0], data[-1, 0], len(data[:,0]))  
O2 = data[:, 1]
CO2 = data[:, 2]
caliper = data[:, 3]
disk = data[:, 4]
ambient = data[:, 5]

#%%


mainLog = "MornMain.CSV"

dataMain = np.genfromtxt(mainLog, delimiter=",", skip_header=10)

# To align the temp and air data with the speed data
offset = -30 # seconds
plotIndex = 4300 # How many datapoints to plot 

rawTime = dataMain[:, 1] 
rawTime = (rawTime - rawTime[0])/1000 + offset
speed = dataMain[:, 4]/3.6

hallspeed = dataMain[:, 5]/3.6
accelForce = dataMain[:, 10]
propAcc = dataMain[:, 14]
powerMain = dataMain[:, 8]

simSpeed = dataMain[:, 7]

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
plt.title("Tues Sept 15")

ax2.plot(rawTime, speed*3.6, label='Speed', color='g')
ax2.plot(rawTime, simSpeed, label='SimSpeed', color='b')
ax2.set_ylabel("Speed (m/s)",fontsize=12)


ax.plot(rawTime, moving_averageV3(powerMain, 5), label='Power', color='r')

#ax.plot((rawTime[:-1]+rawTime[1:])/2, filtForce, label='GPS Diff', color='m')
#ax.plot((rawTime[:-1]+rawTime[1:])/2, hallforce, label='Hall Diff', color='y')
#ax.plot(rawTime, accelForce, label='Force', color='r')
#ax.plot(rawTime, propAcc, label='Force', color='y')
ax.set_ylabel("Force [N]",fontsize=12)
ax.set_ylabel("Power [W]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')


#%% RPM and Speed versus Distance


plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Distance [m]')
plt.title("Tues Sept 15")

ax2.plot(8045-disp, speed*3.6, label='Speed', color='g')
#ax2.plot(rawTime, simSpeed, label='SimSpeed', color='b')
ax2.set_ylabel("Speed (m/s)",fontsize=12)

rpm = dataMain[:, 9]
ax.plot(8045-disp, rpm, label='Cadence', color='r')

#ax.plot((rawTime[:-1]+rawTime[1:])/2, filtForce, label='GPS Diff', color='m')
#ax.plot((rawTime[:-1]+rawTime[1:])/2, hallforce, label='Hall Diff', color='y')
#ax.plot(rawTime, accelForce, label='Force', color='r')
#ax.plot(rawTime, propAcc, label='Force', color='y')
ax.set_ylabel("Force [N]",fontsize=12)
ax.set_ylabel("Power [W]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')





# %% Speed and Acceleration Force


plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Tues Sept 15")

ax2.plot(rawTime, speed*3.6, label='Speed', color='g')
#ax2.plot(rawTime, simSpeed, label='SimSpeed', color='b')
ax2.set_ylabel("Speed (m/s)",fontsize=12)




ax.plot((rawTime[:-1]+rawTime[1:])/2, filtForce, label='GPS Diff', color='m')
ax.plot((rawTime[:-1]+rawTime[1:])/2, hallforce, label='Hall Diff', color='y')
ax.plot(rawTime, accelForce, label='Force', color='r')

ax.set_ylabel("Force [N]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')


# %%
# CO2 PPM and Speed [m/s]
plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Thurs Sept 17")

#ax2.plot(rawTime, speed, label='Speed', color='g')
#ax2.set_ylabel("Speed (m/s)",fontsize=12)


ax2.plot(time, caliper, label='Caliper', color='g')
ax2.plot(time, disk, label='Disk', color='r')
#ax.plot(time, ambient, label='Ambient', color='y')
ax2.set_ylabel("Temperature [C]",fontsize=12)


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


# %% Gear Calcs

rpm = dataMain[:, 9]

gear = dataMain[:, 18]
calcSpeed = dataMain[:, 19]


plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Mon Sept 14")

ax2.plot(rawTime, speed*3.6, label='Speed', color='g')
ax2.plot(rawTime, rpm, label='Cadence', color='b')
ax2.plot(rawTime, calcSpeed, label='Cad Speed', color='y')
ax2.set_ylabel("Speed (m/s)",fontsize=12)

ax.plot(rawTime, gear, label='Gear', color='r')

#ax.plot((rawTime[:-1]+rawTime[1:])/2, filtForce, label='Force', color='b')
#ax.plot((rawTime[:-1]+rawTime[1:])/2, hallforce, label='Force', color='m')
#ax.plot(rawTime, accelForce, label='Force', color='r')
#ax.plot(rawTime, propAcc, label='Force', color='y')
#ax.set_ylabel("Force [N]",fontsize=12)


ax.set_ylabel("Gear [#]",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')





# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# MONDAY DATA

filename = "MondayMorn.CSV"
data = np.genfromtxt(filename, delimiter=",", skip_header=1)

# Interpolate time since lost precision (assume constant sampling frequency)
time = np.linspace(data[0, 0], data[-1, 0], len(data[:,0]))  
O2 = data[:, 1]
CO2 = data[:, 2]
caliper = data[:, 3]
disk = data[:, 4]
ambient = data[:, 5]

mainLog = "MondayMornMain.CSV"
#mainLog = "LOG5WithGearPlot.CSV"
#mainLog = "SLG381.CSV"
dataMain = np.genfromtxt(mainLog, delimiter=",", skip_header=10)

# To align the temp and air data with the speed data
offset = -20 # seconds
plotIndex = 4300 # How many datapoints to plot 

rawTime = dataMain[:, 1] 
rawTime = (rawTime - rawTime[0])/1000 + offset
speed = dataMain[:, 4]/3.6

hallspeed = dataMain[:, 5]/3.6
accelForce = dataMain[:, 10]
propAcc = dataMain[:, 14]
powerMain = dataMain[:, 8]

simSpeed = dataMain[:, 7]

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


# %%


# CO2 PPM and Speed [m/s]
plt.figure()
ax = plt.gca()
ax2 = ax.twinx()
plt.axis('normal')
ax.grid(True)
ax.set_xlabel('Time [s]')
plt.title("Tues Sept 15")

ax2.plot(rawTime, speed, label='Speed Taped', color='g', linestyle='--')
#ax2.set_ylabel("Speed (m/s)",fontsize=12)

ax.plot(time, moving_averageV3(CO2, 50), label='CO2 Taped', color='b', linestyle='--')
#ax.set_ylabel("ppm",fontsize=12)

ax.legend(loc= 'left')
ax2.legend(loc= 'right')



