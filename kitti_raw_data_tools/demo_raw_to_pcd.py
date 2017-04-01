"""Example of pykitti.raw usage."""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import os

import pykitti

__author__ = "Lee Clement"
__email__ = "lee.clement@robotics.utias.utoronto.ca"

# Change this to the directory where you store KITTI data
basedir = '/Users/bird/Downloads/RSdata/dd/raw_data_downloader'

# Specify the dataset to load
date = '2011_09_26'
drive = '0018'

# Optionally, specify the frame range to load
frame_range = range(0, 144, 1)

# Load the data
# dataset = pykitti.raw(basedir, date, drive)
dataset = pykitti.raw(basedir, date, drive, frame_range)

# Load some data
dataset.load_calib()        # Calibration data are accessible as named tuples
dataset.load_timestamps()   # Timestamps are parsed into datetime objects
dataset.load_velo()         # Each scan is a Nx4 array of [x,y,z,reflectance]

# Display some of the data
np.set_printoptions(precision=4, suppress=True)



def string2timestamp(d):
    t = d.timetuple()  
    timeStamp = int(time.mktime(t))  
    timeStamp = float(str(timeStamp) + str("%06d" % d.microsecond))/1000  
    return timeStamp  

output_path = os.path.join(dataset.data_path, 'pcd')
print output_path
if not os.path.exists(output_path):  
   os.makedirs(output_path)
for i in frame_range:
	#filename = string2timestamp(dataset.timestamps[i])
	filename = str(i) + ".pcd"
	print ("the output file name is:%r." %filename)

	output = dataset.velo[i][np.where(dataset.velo[i][:,0]>0)]
	#output = output[np.where(output[:,2]>-2)]
	output[:,3] = map(lambda x : 40 - math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]),output)
	dis = output[:,3].copy()
	output[:,3] = map(lambda x : 1,output)

	temp = (dataset.calib.T_cam0_velo.dot(output[:,0:4].T)).T
	temp = temp[:,0:4]
	temp = temp[np.where(temp[:,2]>0)]
	# temp = (dataset.calib.FINAL_rect_cam0_velo.dot(output[:,0:4].T)).T
# 	temp = np.array(map(lambda x : [x[0]/x[2],x[1]/x[2],1],temp))
# 	temp[:,2] = dis
# 	temp = temp[np.where(temp[:,0]>0)]
# 	temp = temp[np.where(temp[:,0]<1242)]
# 	temp = temp[np.where(temp[:,1]>0)]
# 	temp = temp[np.where(375>temp[:,1])]
# 	temp[:, 1] = -temp[:, 1]
	#print temp.shape
	# f3 = plt.figure()
# 	ax3 = f3.add_subplot(111)
# 	ax3.scatter(temp[:, 0],-temp[:, 1],c=temp[:, 2]/20,cmap='gray')
# 	ax3.set_title('2d Velodyne scan (subsampled)')
# 	plt.show()

	count = temp.shape[0]
	pcdfile = open(output_path + '/' + filename,"w")

# 
	list = ['# .PCD v.5 - Point Cloud Data file format\n','VERSION .5\n','FIELDS x y z\n','SIZE 4 4 4\n','TYPE F F F\n','COUNT 1 1 1\n']
# 
	pcdfile.writelines(list)
	pcdfile.write('WIDTH ') 
	pcdfile.write(str(count))
	pcdfile.write('\nHEIGHT ')
	pcdfile.write(str(1)) 
	pcdfile.write('\nVIEWPOINT 0 0 0 1 0 0 0')
	pcdfile.write('\nPOINTS ')
	pcdfile.write(str(count))
	pcdfile.write('\nDATA ascii\n')
	for j in range(temp.shape[0]):
		pcdfile.write(str(temp[j,0]) + ' ' + str(temp[j,1]) + ' ' + str(temp[j,2])+'\n')
	pcdfile.close()