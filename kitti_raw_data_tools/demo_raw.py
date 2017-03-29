"""Example of pykitti.raw usage."""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math

import pykitti

__author__ = "Lee Clement"
__email__ = "lee.clement@robotics.utias.utoronto.ca"

# Change this to the directory where you store KITTI data
basedir = '/Users/bird/Downloads/RSdata/dd/raw_data_downloader'

# Specify the dataset to load
date = '2011_09_26'
drive = '0011'

# Optionally, specify the frame range to load
frame_range = range(0, 20, 5)

# Load the data
# dataset = pykitti.raw(basedir, date, drive)
dataset = pykitti.raw(basedir, date, drive, frame_range)

# Load some data
dataset.load_calib()        # Calibration data are accessible as named tuples
dataset.load_timestamps()   # Timestamps are parsed into datetime objects
dataset.load_oxts()         # OXTS packets are loaded as named tuples
dataset.load_gray()         # Left/right images are accessible as named tuples
dataset.load_rgb()          # Left/right images are accessible as named tuples
dataset.load_velo()         # Each scan is a Nx4 array of [x,y,z,reflectance]

# Display some of the data
np.set_printoptions(precision=4, suppress=True)
print('\nDrive: ' + str(dataset.drive))
print('\nFrame range: ' + str(dataset.frame_range))

print('\nIMU-to-Velodyne transformation:\n' + str(dataset.calib.T_velo_imu))
print('\nGray stereo pair baseline [m]: ' + str(dataset.calib.b_gray))
print('\nRGB stereo pair baseline [m]: ' + str(dataset.calib.b_rgb))

print('\nFirst timestamp: ' + str(dataset.timestamps[0]))
print('\nSecond IMU pose:\n' + str(dataset.oxts[1].T_w_imu))

f, ax = plt.subplots(2, 2, figsize=(15, 5))
ax[0, 0].imshow(dataset.gray[0].left, cmap='gray')
ax[0, 0].set_title('Left Gray Image (cam0)')

ax[0, 1].imshow(dataset.gray[0].right, cmap='gray')
ax[0, 1].set_title('Right Gray Image (cam1)')

ax[1, 0].imshow(dataset.rgb[0].left)
ax[1, 0].set_title('Left RGB Image (cam2)')

ax[1, 1].imshow(dataset.rgb[0].right)
ax[1, 1].set_title('Right RGB Image (cam3)')





output = dataset.velo[0][np.where(dataset.velo[0][:,0]>0)]
#output = output[np.where(output[:,2]>-2)]
output[:,3] = map(lambda x : 40 - math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]),output)
print "dis:",output
dis = output[:,3].copy()
output[:,3] = map(lambda x : 1,output)

print dataset.calib.FINAL_rect_cam0_velo

print (dataset.calib.T_cam0_velo.dot(output[:,0:4].T)).T
temp = (dataset.calib.FINAL_rect_cam0_velo.dot(output[:,0:4].T)).T
temp = np.array(map(lambda x : [x[0]/x[2],x[1]/x[2],1],temp))
temp[:,2] = dis
temp = temp[np.where(temp[:,0]>0)]
temp = temp[np.where(temp[:,0]<1242)]
temp = temp[np.where(temp[:,1]>0)]
temp = temp[np.where(375>temp[:,1])]
print temp,temp.shape


#print output
# Plot every 100th point so things don't get too bogged down
f2 = plt.figure()
ax2 = f2.add_subplot(111, projection='3d')
velo_range = range(0, output.shape[0], 5)
ax2.scatter(output[velo_range, 0],
            output[velo_range, 1],
            output[velo_range, 2],
            c=output[velo_range, 3],
            cmap='gray')
ax2.set_title('Third Velodyne scan (subsampled)')

f3 = plt.figure()
ax3 = f3.add_subplot(111)
ax3.scatter(temp[:, 0],
            -temp[:, 1],
            c=temp[:, 2]/20,
            cmap='gray')
ax3.set_title('2d Velodyne scan (subsampled)')

plt.show()
