
import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D


# Read input imu data
tp, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, magneto_x, magneto_y, magneto_z = np.loadtxt("data/imu_drone_stationary_2.dat", unpack = True)

# Print some stats
print "Roll : %6.4f%s%6.4f degrees" %(roll.mean(), '+/-', roll.std())
print "Pitch : %6.4f%s%6.4f degrees " %(pitch.mean(), '+/-', pitch.std())
print "Yaw : %6.4f%s%6.4f degrees" %(yaw.mean(), '+/-', yaw.std())
print ""

print "Gyro_X : %6.4f%s%6.4f degrees/sec" %(gyro_x.mean(), '+/-', gyro_x.std())
print "Gyro_Y : %6.4f%s%6.4f degrees/sec" %(gyro_y.mean(), '+/-', gyro_y.std())
print "Gyro_Z : %6.4f%s%6.4f degrees/sec" %(gyro_z.mean(), '+/-', gyro_z.std())
print ""

print "Magneto_X : %6.4f%s%6.4f Gauss" %(magneto_x.mean(), '+/-', magneto_x.std())
print "Magneto_Y : %6.4f%s%6.4f Gauss" %(magneto_y.mean(), '+/-', magneto_y.std())
print "Magneto_Z : %6.4f%s%6.4f Gauss" %(magneto_z.mean(), '+/-', magneto_z.std())

# Display roll, pitch and yaw variation with time in seconds
plt.figure(figsize = (10,7))
plt.xlabel("Time (secs)", fontsize = 14)
plt.ylabel("Angle (degrees)", fontsize = 14)
plt.title("Microstrain IMU: Euler Angles", fontsize = 14)
plt.plot(tp, roll, label = 'Roll', linewidth = 2)
plt.plot(tp, pitch, label = 'Pitch', linewidth = 2)
plt.plot(tp, yaw, label = 'Yaw', linewidth = 2)
plt.grid()
plt.legend()
plt.show()

plt.figure(figsize = (10,7))
plt.xlabel("Time (secs)", fontsize = 14)
plt.ylabel("Angular Velocity (degrees/sec)", fontsize = 14)
plt.title("Microstrain IMU: Gyro", fontsize = 14)
plt.plot(tp, gyro_x, label = 'Gyro_X', linewidth = 2)
plt.plot(tp, gyro_y, label = 'Gyro_Y', linewidth = 2)
plt.plot(tp, gyro_z, label = 'Gyro_Z', linewidth = 2)
plt.grid()
plt.legend()
plt.show()

fig = plt.figure(figsize = (10,7))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(magneto_x, magneto_y, magneto_z)
ax.set_xlabel('Magneto_X (Gauss)', fontsize = 14)
ax.set_ylabel('Magneto_Y (Gauss)', fontsize = 14)
ax.set_zlabel('Magneto_Z (Gauss)', fontsize = 14)
ax.set_title("Microstrain IMU: Magnetometer", fontsize = 14)
plt.show()