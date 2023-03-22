#!/usr/local/bin/python3
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('ATS_Log_171119_13h24m09s')
plt.xlabel('Time Relative to Launch (s)')
plt.ylabel('Altitude (ft)')
plt.plot(data[:,0]-2425, data[:, 1], '-')
plt.xlim((-10,112))
#plt.savefig('agl_v_time.png')
plt.show()
