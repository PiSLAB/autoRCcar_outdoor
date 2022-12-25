#!/usr/bin/env

import matplotlib.pyplot as plt
import numpy as np
import math

def quat2eulr(quat):
    quat = quat / np.linalg.norm(quat)

    # w,x,y,z 순서
    a = quat[0]
    b = quat[1]
    c = quat[2]
    d = quat[3]

    A = 2*(a*b + c*d)
    B = a*a - b*b - c*c + d*d
    C = 2*(b*d - a*c)
    D = 2*(a*d + b*c)
    E = a*a + b*b - c*c - d*d

    phi = math.atan2(A,B)  
    theta = math.asin(-C)
    psi = math.atan2(D,E)

    euler = [phi, theta, psi] # rad

    return euler

# # Example 데이터
# csv_data = np.loadtxt('output.csv', delimiter=',')
# time = csv_data[:,0]
# pos_x = csv_data[:,4]
# pos_y = csv_data[:,5]
# pos_z = csv_data[:,6]
# vel_x = csv_data[:,7]
# vel_y = csv_data[:,8]
# vel_z = csv_data[:,9]
# quat_w = csv_data[:,10]
# quat_x = csv_data[:,11]
# quat_y = csv_data[:,12]
# quat_z = csv_data[:,13]
# num = len(csv_data[:,0])

# NavTest 데이터
csv_data = np.loadtxt('navtest_output.csv', delimiter=',')
time0 = csv_data[:,0]
time1 = csv_data[:,1]
time = time0 + time1*1e-9
pos_x = csv_data[:,5]
pos_y = csv_data[:,6]
pos_z = csv_data[:,7]
vel_x = csv_data[:,8]
vel_y = csv_data[:,9]
vel_z = csv_data[:,10]
quat_w = csv_data[:,14]
quat_x = csv_data[:,11]
quat_y = csv_data[:,12]
quat_z = csv_data[:,13]
num = len(csv_data[:,0])

plt.plot(pos_y, pos_x)
plt.axis('equal')
plt.xlabel("East [m]")
plt.ylabel("North [m]")
plt.title("Horizontal position")
plt.show()

plt.subplot(3,1,1)
plt.plot(time, vel_x)
plt.ylabel(r'$V_n [m/s]$')
plt.subplot(3,1,2)
plt.plot(time, vel_y)
plt.ylabel(r'$V_e [m/s]$')
plt.subplot(3,1,3)
plt.plot(time, vel_z)
plt.xlabel('time [sec]')
plt.ylabel(r'$V_d [m/s]$')
plt.show()

rpy = np.zeros((num,3))
for i in range(num):
    quat = [quat_w[i], quat_x[i], quat_y[i], quat_z[i]]
    euler = quat2eulr(quat)
    rpy[i][0] = euler[0]
    rpy[i][1] = euler[1]
    rpy[i][2] = euler[2]

plt.subplot(3,1,1)
plt.plot(time, rpy[:,0]*180/math.pi)
plt.ylabel('roll [deg]')
plt.subplot(3,1,2)
plt.plot(time, rpy[:,1]*180/math.pi)
plt.ylabel('pitch [deg]')
plt.subplot(3,1,3)
plt.plot(time, rpy[:,2]*180/math.pi)
plt.xlabel('time [sec]')
plt.ylabel('yaw [deg]')
plt.show()
