"""Example of pykitti.raw usage."""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import os,sys

import pykitti

#python  demo_raw_to_pcd.py /media/bird/GSP1RMCULXF/didi/raw_data_downloader 2011_09_26_drive_0119
#/media/bird/GSP1RMCULXF/didi/raw_data_downloader/2011_09_26/2011_09_26_drive_0018_sync/image_00/data/0000000000.png
# Change this to the directory where you store KITTI data

basedir = sys.argv[1] #'/media/bird/GSP1RMCULXF/didi/raw_data_downloader'
# Specify the dataset to load
date = sys.argv[2].split('_drive_',-1)[0] #'2011_09_26'
drive = sys.argv[2].split('_drive_',-1)[1] #'0005'

output_path = os.path.join(basedir, date)
output_path = os.path.join(output_path, date + "_drive_" + drive + "_sync")
output_path = os.path.join(output_path, 'velodyne_points/data')




# Optionally, specify the frame range to load
count = 0
for parent,dirnames,filenames in os.walk(output_path):    
	for filename in filenames:
		count+=1 
print count
frame_range = range(0, count, 2)

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
	i = i/2
	output = dataset.velo[i][np.where(dataset.velo[i][:,0]>0)]
	#output = output[np.where(output[:,2]>-2)]
	output[:,3] = map(lambda x : 40 - math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]),output)
	dis = output[:,3].copy()
	output[:,3] = map(lambda x : 1,output)


	temp = output[:,0:4]
	temp = temp[np.where(temp[:,0]>-2)]



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
