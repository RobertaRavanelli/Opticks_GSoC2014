import os
import numpy as np
import pylab as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import cm 
from matplotlib.mlab import griddata
import matplotlib.pyplot as pp
import scipy as sp
import scipy.interpolate

path = os.path.realpath(__file__)[0:-12]  
os.chdir(path)
plt.close("all")

building_number = 245

########  RETRIEVE POINT CLOUD DATA ########################
#for i in xrange(110, 120):  
for i in xrange(120, 130):    
 print 'building ' + str(i)

 point_cloud = np.genfromtxt(path+'building_' +str(i)+'.txt', skip_header=1) # COLUMN 0 IS I (ROW IMAGE VOORDINATE) AND COLUMNN 1 IS J (COLUMN IMAGE VOORDINATE) 
 x_pc = point_cloud[:,2] # X
 y_pc = point_cloud[:,3] # Y
 z_pc = point_cloud[:,4] # Z

 inliers = np.genfromtxt(path+'buildings_inliers.txt', skip_header=i, skip_footer = building_number-i)#, skip_header=2, skip_footer=1)
 print inliers
 
 x_inliers= np.zeros(len(inliers))
 y_inliers= np.zeros(len(inliers))
 z_inliers= np.zeros(len(inliers))
 for j in xrange(0,len(inliers),1):
    x_inliers[j] = x_pc[int(inliers[j])]
    y_inliers[j] = y_pc[int(inliers[j])]
    z_inliers[j] = z_pc[int(inliers[j])]

 #plane_parameters = plane_parameters[i]
 plane_parameters = np.genfromtxt(path+'buildings_parameters.txt')[i]
 print plane_parameters
 x_plane = np.array([min(x_inliers), max(x_inliers)])
 y_plane = np.array([min(y_inliers), max(y_inliers)])
 xx, yy = np.meshgrid(x_plane, y_plane)
 z_plane = (-plane_parameters[0] * xx - plane_parameters[1] * yy - plane_parameters[3]) * 1. /plane_parameters[2]
 #z_plane = (-plane_parameters[0] * x_pc - plane_parameters[1] * y_pc - plane_parameters[3]) * 1. /plane_parameters[2]

 fig=plt.figure('point cloud building'+str(i))
 #ax = fig.add_subplot(111, projection='3d')
 ax = fig.add_subplot(211, projection='3d')
 ax.scatter(x_pc, y_pc, z_pc, c=z_pc, marker='o', cmap=plt.cm.jet,label='points')
 ax.set_zlim(0,max(z_pc)) # to avoid the display of bad value
 #ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='^', s=150, label='inliers')
 #plt.show()
 ax.set_xlim(min(x_pc)-2,max(x_pc)+2)
 ax.set_ylim(min(y_pc)-2,max(y_pc)+2)
 ax = fig.add_subplot(212, projection='3d')
 #ax.scatter(x_pc, y_pc, z_pc, c='r', marker='o',label='points')
 #ax.plot_surface(xx, yy, z_plane, color='b')
 #ax.plot_surface(x_pc, y_pc, z_plane, color='b')
 #ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='o', s=150, label='inliers')
 ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='o',  label='inliers')
# ax = fig.add_subplot(313, projection='3d')
 #ax.plot_surface(xx, yy, z_plane, color='b')
 ax.set_xlim(min(x_pc)-2,max(x_pc)+2)
 ax.set_ylim(min(y_pc)-2,max(y_pc)+2)
 ax.set_zlim(0,max(z_pc))
 plt.show()
 
 
print 'finito di disegnare'
