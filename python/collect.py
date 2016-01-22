"""
    Read Euler angles from Microstrain IMU and write to an outfile for
    further processing and plotting.
"""

import os, time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from imu_1_2 import IMU


# Outpath and other input variables
outpath = "/home/ubuntu/Documents/imu_data" 
fname = "imu_drone_propeller_1.dat"
data_points = 500

# Create an IMU objects
imu = IMU()

t0 = time.time()
outfile = os.path.join(outpath, fname)
tp = np.zeros(data_points, dtype = np.float64)
with open(outfile, 'w') as fd:
    for i in range(data_points):
        #roll, pitch, yaw = imu.get_euler_angles()
        #gyro_x, gyro_y, gyro_z = imu.get_scaled_gyro()
        #magneto_x, magneto_y, magneto_z = imu.get_scaled_magneto()
        roll, pitch, yaw, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, magneto_x, magneto_y, magneto_z = imu.get_uav() 
        tp[i] = time.time()
        elapsed_time = tp[i] - t0
        fd.write('%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s%6.4f%s' %(elapsed_time, '\t', roll, '\t', pitch, '\t', yaw, '\t', accl_x, '\t', accl_y, '\t', accl_z, '\t', gyro_x, '\t', gyro_y, '\t', gyro_z, '\t', magneto_x, '\t', magneto_y, '\t', magneto_z, '\n'))
