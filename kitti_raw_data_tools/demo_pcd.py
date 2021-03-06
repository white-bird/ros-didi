"""Example of pykitti.raw usage."""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math,sys
import matplotlib.image as mpimg

import pykitti


# Change this to the directory where you store KITTI data

filename = sys.argv[1]

RT1 = [[7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03],[1.480249e-02,7.280733e-04,-9.998902e-01,-7.631618e-02],[9.998621e-01,7.523790e-03,1.480755e-02,-2.717806e-01],[0.,0.,0.,1]] #kitti
RT2 = [[ 7.5588315892942126e-02, 9.3538503935107931e-02,
	9.9274213911873932e-01, 9.3900513464616231e-01],[
	-9.9673508097596830e-01, 3.5426574795548005e-02,
	7.2554366857447206e-02, 1.2554482398397798e-01],[
	-2.8382826714377640e-02, -9.9498517882420601e-01,
	9.5910943420781658e-02, -8.3035952544017211e-01],[ 0., 0., 0., 1. ]] #didi
P = [[7.215377e+02,0.000000e+00,6.095593e+02,0.000000e+00],
	[0.000000e+00,7.215377e+02,1.728540e+02,0.000000e+00],
	[0.000000e+00,0.000000e+00,1.000000e+00,0.000000e+00]]
P2 = [[2200.899902, 0.000000, 560.455808, 0.000000], [0.000000, 2218.906494, 447.621656, 0.000000], [0.000000, 0.000000, 1.000000, 0.000000]]
P3 = [[1345.200806, 0.000000, 429.549312, 0.000000], [0.000000, 1353.838257, 369.393325, 0.000000], [0.000000, 0.000000, 1.000000, 0.000000]]
# Load the data
f = open(filename)
line = f.readline()
while line:
	if not line.strip():
		line = f.readline()
		continue
	if "ascii" in line.strip():
		line = f.readline()
		break
	line = f.readline()

np.set_printoptions(precision=4, suppress=True)
point_cloud = []
while line:
	if not line.strip():
		line = f.readline()
		continue
	xyz = line.strip().split(' ', -1)
	point_cloud.append([xyz[0],xyz[1],xyz[2], 1])
	line = f.readline()


temp = (np.array(RT1).dot(np.array(point_cloud).astype(float).T)).T


#temp = np.array(point_cloud).astype(float)
#temp = (np.array(RT1).dot(np.array(point_cloud).astype(float).T)).T

temp = temp[np.where(temp[:,2]>0)]
temp = temp[np.where(temp[:,2]<80)]

#temp = np.array(point_cloud).astype(float)
xyz_info = temp.copy()
xyz_info[:,3] = map(lambda x : 40 - math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]),xyz_info)

#print "dis:",xyz_info
#print temp

temp2d = (np.array(P).dot(np.array(temp).T)).T

h = 375
w = 1242

temp2d = np.array(map(lambda x : [x[0]/x[2],x[1]/x[2],1],temp2d))
temp2d[:,2] = xyz_info[:,3]
xyz_info2d = xyz_info.copy()
x1 = 0;
x2 = w;
y1 = 0;
y2 = h;

#0.295,0.577,0.146,0.308
#0.325,0.634,0.161,0.276

print x1,x2,y1,y2

xyz_info2d = xyz_info2d[np.where(temp2d[:,0]>x1)]
temp2d = temp2d[np.where(temp2d[:,0]>x1)]
xyz_info2d = xyz_info2d[np.where(temp2d[:,0]<x2)]
temp2d = temp2d[np.where(temp2d[:,0]<x2)]
xyz_info2d = xyz_info2d[np.where(temp2d[:,1]>y1)]
temp2d = temp2d[np.where(temp2d[:,1]>y1)]
xyz_info2d = xyz_info2d[np.where(y2>temp2d[:,1])]
temp2d = temp2d[np.where(y2>temp2d[:,1])]





print temp2d.shape
print xyz_info2d.shape

temp3d = temp


f3 = plt.figure()
ax3 = f3.add_subplot(111)
ax3.scatter(temp2d[:, 0],
            -temp2d[:, 1],
            c=temp2d[:, 2]/20,
            cmap='gray')
ax3.set_title('2d Velodyne scan (subsampled)')

img = np.array([255,255,255] * (w + 10) * (h + 30)).reshape(h + 30, (w + 10), 3)
for i in range(temp2d.shape[0]):
	x = int(temp2d[i,0])
	y = int(temp2d[i,1])
	rx = 2;ry = 4
	img[y:y+ry,x:x+rx,0]=np.array([((xyz_info2d[i,0]*15+87)).clip(1,255)]*ry*rx).reshape(ry,rx)
	img[y:y+ry,x:x+rx,1]=np.array([((-xyz_info2d[i,1]*60+150)).clip(1,255)]*ry*rx).reshape(ry,rx)
	img[y:y+ry,x:x+rx,2]=np.array([((xyz_info2d[i,2]*7+0)).clip(1,255)]*ry*rx).reshape(ry,rx)
	#print x,y,xyz_info2d[i,:]
	
	
f, ax = plt.subplots(2, 2, figsize=(15, 5))
ax[0, 0].imshow(np.apply_along_axis(lambda x : [x[0]] * 3,2,img))
ax[0, 0].set_title('Left Gray Image (cam0)')

ax[0, 1].imshow(np.apply_along_axis(lambda x : [x[1]] * 3,2,img))
ax[0, 1].set_title('Right Gray Image (cam1)')

ax[1, 0].imshow(np.apply_along_axis(lambda x : [x[2]] * 3,2,img))
ax[1, 0].set_title('Left RGB Image (cam2)')

ax[1, 1].imshow(img)
ax[1, 1].set_title('Left RGB Image (cam3)')
mpimg.imsave("./output.jpg",img)
plt.show()
