import os
import numpy as np
import pylab as plt
import mpl_toolkits.mplot3d.axes3d as p3


path = os.path.realpath(__file__)[0:-12]   
os.chdir(path)
plt.close("all")

building_number = len(np.genfromtxt(path+'Number_of_RANSAC_applications.txt') )#246

########  RETRIEVE POINT CLOUD DATA ########################
 
for i in xrange(230, 235): 
 print 'building ' + str(i)
 cont = int (np.genfromtxt(path+'Number_of_RANSAC_applications.txt')[i,1]+1) # the total times that RANSAC was applied (IT DEPENDS BY THE SINGLE BUILDING) 8 FOR 230 and 116, 5 FOR 121, 4 for108

 point_cloud = np.genfromtxt(path+'building_' +str(i)+'.txt', skip_header=1) # COLUMN 0 IS I (ROW IMAGE VOORDINATE) AND COLUMNN 1 IS J (COLUMN IMAGE VOORDINATE) 
 x_pc = point_cloud[:,2] # X
 y_pc = point_cloud[:,3] # Y
 z_pc = point_cloud[:,4] # Z

 inliers = np.genfromtxt(path+'buildings_inliers.txt', skip_header=i, skip_footer = building_number-i-1)#, skip_header=2, skip_footer=1)

 
 x_inliers= np.zeros(len(inliers))
 y_inliers= np.zeros(len(inliers))
 z_inliers= np.zeros(len(inliers))
 for j in xrange(0,len(inliers),1):
    x_inliers[j] = x_pc[int(inliers[j])]
    y_inliers[j] = y_pc[int(inliers[j])]
    z_inliers[j] = z_pc[int(inliers[j])]


 #plane_parameters = plane_parameters[i]
 plane_parameters = np.genfromtxt(path+'buildings_parameters.txt')[i]
 #print plane_parameters
 x_plane = np.array([min(x_inliers), max(x_inliers)])
 y_plane = np.array([min(y_inliers), max(y_inliers)])
 xx, yy = np.meshgrid(x_plane, y_plane)
 z_plane = (-plane_parameters[0] * xx - plane_parameters[1] * yy - plane_parameters[3]) * 1. /plane_parameters[2]
 #z_plane = (-plane_parameters[0] * x_pc - plane_parameters[1] * y_pc - plane_parameters[3]) * 1. /plane_parameters[2]
 

 fig = plt.figure('point cloud building'+str(i))
 #ax = fig.add_subplot(111, projection='3d')
 ax = fig.add_subplot(211, projection='3d')
 ax.scatter(x_pc, y_pc, z_pc, c=z_pc, marker='o', cmap=plt.cm.jet,label='points')
 ax.set_zlim(0,max(z_pc)) # to avoid the display of bad value
 #ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='^', s=150, label='inliers')
 #plt.show()
 ax.set_xlim(min(x_pc)-2,max(x_pc)+2)
 ax.set_ylim(min(y_pc)-2,max(y_pc)+2)
 #plt.axis('equal')
 ax = fig.add_subplot(212, projection='3d')
 #ax.scatter(x_pc, y_pc, z_pc, c='r', marker='o',label='points')
 #ax.plot_surface(xx, yy, z_plane, color='b')
 #ax.plot_surface(x_pc, y_pc, z_plane, color='b')
 #ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='o', s=150, label='inliers')
 plane_parameters_boh = np.genfromtxt(path+'plane_parameters_building_'+str(i)+'.txt')
 color_array = np.array(['b', 'g','brown', 'white', 'black', 'purple', 'c'])
 for z in xrange(0, min(cont-1,len(color_array))):  
     plane_parameters_boh = np.genfromtxt(path+'plane_parameters_building_'+str(i)+'.txt',skip_header = z, skip_footer = cont-2-z )
     inliers_boh = np.genfromtxt(path+'Inliers_building_'+str(i)+'.txt', skip_header = z, skip_footer = cont-2-z )#building 116 and 230
     #print inliers_boh 
     x_inliers_boh = np.zeros(len(inliers_boh))
     y_inliers_boh = np.zeros(len(inliers_boh))
     z_inliers_boh = np.zeros(len(inliers_boh))
     for j in xrange(0,len(inliers_boh),1):
       x_inliers_boh[j] = x_pc[int(inliers_boh[j])]
       y_inliers_boh[j] = y_pc[int(inliers_boh[j])]
       z_inliers_boh[j] = z_pc[int(inliers_boh[j])]
     ax.scatter(x_inliers_boh, y_inliers_boh, z_inliers_boh, c= color_array[z], marker='o',  label="inliers " +str(z))
      # the code below draws the plane normal
     ax.plot([np.mean(x_inliers_boh), np.mean(x_inliers_boh) + 35 * plane_parameters_boh[0]], [np.mean(y_inliers_boh), np.mean(y_inliers_boh) + 35 * plane_parameters_boh[1]],[np.mean(z_inliers_boh), np.mean(z_inliers_boh) + 35 * plane_parameters_boh[2]],color = color_array[z])
     ax.scatter(np.mean(x_inliers_boh) + 35 * plane_parameters_boh[0],  np.mean(y_inliers_boh) + 35 * plane_parameters_boh[1], np.mean(z_inliers_boh) + 35 * plane_parameters_boh[2], color = color_array[z], marker = "^")
     #ax.plot_surface(xx, yy, z_plane, color=color_array[z])

 ax.scatter(x_inliers, y_inliers, z_inliers, c='y', marker='o',  label="inliers 0")
 #ax.legend(True)
 
# ax = fig.add_subplot(313, projection='3d')
 #ax.plot_surface(xx, yy, z_plane, color='b')
 ax.set_xlim(min(x_pc)-2,max(x_pc)+2)
 ax.set_ylim(min(y_pc)-2,max(y_pc)+2)
 ax.set_zlim(0,max(z_pc))
 
 #fig2 = fig.gca(projection='3d')
 # the code below draws the plane normal
 ax.plot([np.mean(x_inliers), np.mean(x_inliers) + 35 * plane_parameters[0]], [np.mean(y_inliers), np.mean(y_inliers) + 35 * plane_parameters[1]],[np.mean(z_inliers), np.mean(z_inliers) + 35 * plane_parameters[2]],color = 'y')
 ax.scatter(np.mean(x_inliers) + 35 * plane_parameters[0],  np.mean(y_inliers) + 35 * plane_parameters[1], np.mean(z_inliers) + 35 * plane_parameters[2], color = 'y', marker = "^")

 plt.show()
 

