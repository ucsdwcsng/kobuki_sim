#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 22:00:24 2020

@author: joel
"""
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize as opt
import csv
import argparse
import pdb
parser = argparse.ArgumentParser(description="Measures error of cartographer trajectory of robot moving in a circle w/ drift error, save images of overall & x&y error and overall & x&y trajectories")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")
parser.add_argument('imu_noise', action = 'store', 
		    help = "imu noise stddev")
parser.add_argument('lidar_noise', action = 'store', 
		    help = "lidar noise stddev")
parser.add_argument('config', action = 'store', 
		    help = "config file")
args = parser.parse_args()

base = args.bagfile + '_imu-' + args.imu_noise + '_lidar-' + args.lidar_noise + '_' + args.config

file = '/home/joel/bagfiles/' + args.bagfile + '/tq_' + base + '.csv'
print(file)
#file = '/home/joel/bagfiles/record_files_output_highIMU2/tf_output_highIMU2_footprint_combined.csv'
#%%
#trans is the x and y data points, and times are the times
trans = []
times = []
rotations = []
with open (file) as f:
    coordinates = csv.reader(f)
    for row in coordinates:
        trans.append(list(map(float,row[1:3])))
        times.append(float(row[0]))
        
trans = np.array(trans)
times = np.array(times) - times[0]

#%%
#Select circles and plot them
plt.figure()
selection = np.arange(20,len(times))

plt.plot(trans[selection,0], trans[selection,1],'-')

ax = plt.gca()
ax.set_aspect('equal')
#label every 20th point
for i, xy in enumerate(zip(trans[selection[::20], 0], trans[selection[::20], 1])):
    ax.annotate(str(i), xy)
#plt.title('Data points')    
# %%
#Find center and radius for circle of best fit
#plt.figure()
circ = []
circ_times = []    
circ.append(range(selection[0], selection[len(selection) - 1] +1))
    
center = []
radius = []

for c in circ:
    plt.plot(trans[c,0], trans[c,1],'-')
    x = trans[c,0]
    y = trans[c,1]
    A = np.column_stack((-2*x, -2*y, np.ones((len(x),1))))
    b = (x**2 + y**2)
    sol = np.linalg.lstsq(A, -b)[0]
    center.append(sol[:2])
    radius.append(np.sqrt(-sol[2] + sol[0]**2 + sol[1]**2))

center = np.array(center)
radius = np.array(radius)
plt.plot(trans[selection[0],0],trans[selection[0],1],'x', color = 'g')
plt.plot(trans[selection[len(selection)-1],0],trans[selection[len(selection)-1],1],'x', color = 'r')    
plt.title('Circle Data points')    
#%%
#plot circles based on solved values for center and radius
circ_plot = []
for i, c in enumerate(circ):
    fig = plt.figure()
    plt.plot(trans[c,0], trans[c,1])
    circ_plot.append(plt.Circle(center[i],radius[i], color = 'r', fill = False))
for c_plot in circ_plot:   
    plt.gca().add_artist(c_plot) 
plt.plot(center[:,0], center[:,1], 'x')

plt.gca().set_aspect('equal')    
plt.title('Best Fit Circle: Radius = %0.3f m' %radius[0])    
fig.savefig("/home/joel/bagfiles/" + args.bagfile + "/" + base + "_trajectory.png")  
#%%
#calculate errors 
#radius = [0.026]

angles = np.array([np.arccos((trans[c.start, :2]-cent).dot(trans[c.stop-1, :2]-cent)\
                  /np.linalg.norm(trans[c.start, :2]-cent)\
                  /np.linalg.norm(trans[c.stop-1, :2]-cent))\
                    for c, cent in zip(circ, center)]) #angle between start and stop of input data
#print(angles/np.pi)
start_angles = np.array([np.arccos((trans[c.start, :2]-cent).dot(np.array([1, 0]))\
                  /np.linalg.norm(trans[c.start, :2]-cent)) \
                    for c, cent in zip(circ, center)])


#Determine if moving clockwise or counterclockwise
counterClockwise = np.cross(trans[c.start,:2]-center, trans[c.start+1,:2]-center) > 0

#Determine if the remaining radians after taking into account full revolutions is < than a semicircle
remainingRadians = counterClockwise ^ ((np.cross(trans[c.start,:2]-center, trans[c.stop-1,:2]-center)) > 0)

#Determine if start radian is above the x axis
aboveXAxis = (trans[c.start][1] - center[0][1]) > 0

#Determine number of revolutions


shifted_points = trans - center
inverse_transmatrix = np.array([[shifted_points[c.start][0], shifted_points[c.start][1]], 
				[shifted_points[c.start][1],-shifted_points[c.start][0]]])
#pdb.set_trace()
new_points = [np.linalg.inv(inverse_transmatrix).dot(point)[1] >= 0 for point in shifted_points[c.start:c.stop]]
semi_circles = 0
for i in range(2, len(new_points)-1):
	if new_points[i] != new_points[i-1]:
		semi_circles = semi_circles+1
total_revolutions = int((semi_circles/2))
print(total_revolutions)
#pdb.set_trace()


#radians between start and end angles
if not remainingRadians:
	angles= np.array([angles[0]])
else:
	angles = 2*np.pi - np.array([angles[0]]) 

#radians between positive x axis and start
if aboveXAxis:
	start_angles = np.array([start_angles[0]])
else:
	start_angles = 2*np.pi - np.array([start_angles[0]])

#total radians traveled
if counterClockwise:
	total_rad = 2*np.pi*np.array([total_revolutions + angles[0]/2/np.pi])
else:
	total_rad = -2*np.pi*np.array([total_revolutions + angles[0]/2/np.pi])


rad_per_point =  total_rad/np.array(list(map(len, circ)))
            
#pdb.set_trace()


circ_angles = [((times[rng]-times[rng.start])/(times[rng.stop-1]) - times[rng.start])*tr + sa \
               for rng, tr, sa in zip(circ, total_rad, start_angles)]
#circ_angles = [(times[rng]-times[rng.start])/(times[rng.stop-1]-times[rng.start])*tr + sa
#                for rng, tr, sa in zip(circ, total_rad, start_angles)]

circ_angles_old = ((list(map(np.arange, list(map(len, circ)))))*rad_per_point + start_angles)

sim_points_old = [cent + np.column_stack((rad*np.cos(circ_a), rad*np.sin(circ_a))) \
                  for rad, circ_a, cent in zip(radius, circ_angles, center)]
sim_points = [cent + np.column_stack((rad*np.cos(circ_a), rad*np.sin(circ_a))) \
                  for rad, circ_a, cent in zip(radius, circ_angles_old, center)]
#plt.figure()
#plt.plot(np.array(sim_points))
#plt.title('Simulated points')
#pdb.set_trace()
error = [np.linalg.norm(sim_p - trans[c,:2], axis=1) for sim_p, c in zip(sim_points, circ)]
error_xy = [ np.abs(sim_p - trans[c,:2]) for sim_p, c in zip(sim_points, circ)]
all_error = np.hstack(error)
all_error_xy = np.hstack(error_xy)
error_old = [np.linalg.norm(sim_p - trans[c, :2], axis=1) for sim_p, c in zip(sim_points_old, circ)]
all_error_old = np.hstack(error_old)
#pdb.set_trace()
#%%
#Plot of Cumulative D	timeistribution of Norm Error
plt.figure()
for i in range(len(circ)):
    plt.subplot(1,len(circ),i+1)
    plt.hist(error[i], 50, normed=1, histtype='step', cumulative=True)
    plt.title('CDF of errors with median error = %0.3f' % (np.median(error[i])))    
#%%
#Plot of CDF for XY error
for i in range(len(circ)):
    plt.figure()
    plt.subplot(121)
    plt.hist(error_xy[i][:, 0], 50, normed=1, histtype='bar', cumulative=False)
    plt.title('CDF of X-axis errors with median error = %0.3f' % \
              (np.median(error_xy[i][:, 0])))   
    
    plt.subplot(122)
    plt.hist(error_xy[i][:, 1], 50, normed=1, histtype='bar', cumulative=False)
    plt.title('CDF of Y-axis errors with median error = %0.3f' % \
              (np.median(error_xy[i][:, 1])))
plt.suptitle('Radius = %0.3f m'%radius[0])
# %%
plt.figure()
for err, err_old in zip(error, error_old): 
    plt.plot(err, '-')
    plt.plot(err_old, '--')
plt.legend(range(3))
# %%
plt.figure()
for err, err_old in zip(error, error_old): 
    plt.plot(err, '-')
    plt.plot(err_old, '--')
plt.legend(range(3))

# %% store error from first circle data set 
error_1 = all_error
error_1_xy = all_error_xy
# %% concat current dataset's errors: 
error_2 = all_error
error_2_xy = all_error_xy
two_error = np.append(np.array(error_1), np.array(error_2))
two_error_xy = np.vstack((np.array(error_1_xy), np.array(error_2_xy)))

fig = plt.figure()
plt.hist(two_error, 50, density=1, histtype='step', cumulative=True, \
         linewidth=3)
plt.title('CDF of errors with median error = %0.3f' % (np.median(two_error)))

prop = ['bar', False]
fig.savefig("/home/joel/bagfiles/" + args.bagfile + "/" + base + "_overall_error.png")

fig = plt.figure()
plt.subplot(121)
plt.hist(two_error_xy[:, 0], 50, density=1, histtype = prop[0], cumulative = prop[1], \
         linewidth=3)
plt.title('CDF of X-axis errors with median error = %0.3f' % \
          (np.median(error_xy[i][:, 0])))   

plt.subplot(122)
plt.hist(two_error_xy[:, 1], 50, density=1, histtype = prop[0], cumulative = prop[1], \
         linewidth=3)
plt.title('CDF of Y-axis errors with median error = %0.3f' % \
          (np.median(error_xy[i][:, 1])))
plt.suptitle('Plot of both circles\' datasets')
fig.savefig("/home/joel/bagfiles/" + args.bagfile + "/" + base + '_xy_error.png')

#%%
fig = plt.figure()
sim_points = np.array(sim_points)

plt.subplot(121)
plt.plot(range(selection[0], selection[len(selection)-1]+1), sim_points[0][:,0], color = 'red')
plt.plot(range(selection[0], selection[len(selection)-1]), trans[selection[0]:selection[len(selection)-1],0], color = 'green')
plt.title('X Values')


plt.subplot(122)
plt.plot(range(selection[0], selection[len(selection)-1]+1), sim_points[0][:,1], color = 'red')
plt.plot(range(selection[0], selection[len(selection)-1]), trans[selection[0]:selection[len(selection)-1],1], color = 'green')
plt.title('Y Values')
fig.savefig("/home/joel/bagfiles/" + args.bagfile + "/" + base + "_xy_trajectory.png")
#pdb.set_trace()
plt.figure()
plt.plot(sim_points[0][:,0],sim_points[0][:,1], '-')
ax = plt.gca()
for i, xy in enumerate(zip(sim_points[0][::20,0], sim_points[0][::20,1])):
    ax.annotate(str(i), xy)
#pdb.set_trace()
plt.plot(sim_points[0][0,0],sim_points[0][0,1],'x', color = 'g')
plt.plot(sim_points[0][len(sim_points[0])-1,0],sim_points[0][len(sim_points[0])-1,1],'x', color = 'r')    
plt.title('Sim points')     

print(np.median(error))

inputcsv = "/home/joel/bagfiles/" + args.bagfile + "/output.csv"

csv_reader = csv.reader(open(inputcsv, 'r'))
lines = list(csv_reader)
for row in lines:
	
	if(args.imu_noise == row[0] and args.lidar_noise == row[1]):
		if(args.config == "combined"):
				row[2] = np.median(error)
				print("Edited combined")
		elif(args.config == "imu_only"):
				row[3] = np.median(error)
				print("Edited imu_only")
		elif(args.config == "lidar_only"):
				row[4] = np.median(error)
				print("Edited lidar_only")
#pdb.set_trace()
outputcsv = "/home/joel/bagfiles/"+ args.bagfile + "/output.csv"
print(lines)
csv_writer = csv.writer(open(outputcsv, 'w+'))
csv_writer.writerows(lines)
