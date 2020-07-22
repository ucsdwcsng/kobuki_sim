#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 11 14:39:15 2019

@author: aarun

The following script calculates the best fit circle for the pose data given,
 uses this to calculate the ground truth accuracy of the bot. Currently handles 
 outputs from gmapping and rtabmap. 
"""
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize as opt 
import quaternion
import csv
import argparse
# %%
# Extract translation if gmapping
#file = './data/poses/poses_all_circles.txt'
#trans = []
#times = []
#with open(file) as f:
#    for line in f:
#        if 'time' in line:
#             times.append(float(re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]))
#        elif 'Translation' in line:
#            vals = re.findall(r"[-+]?\d*\.\d+|\d+", line)
#            trans.append(list(map(float, vals)))
#trans = np.array(trans)
#times = np.array(times)

#%%
# Extract translation if rtabmap
#file = '/home/aarun/Research/Data/deep_loc/data/poses/poses_big_circle.txt'
#trans = []
#times = []
#rotations = []
#with open(file) as f:
#   for line in f:
#        vals = re.findall(r"[-+]?\d*\.\d+|\d+", line)
#        trans.append(list(map(float, vals[1:3])))
#        times.append(float(vals[0]))
#        quat = np.array(list(map(float, vals[4:])))[[3, 0, 1, 2]]
#        rotations.append(quaternion.from_float_array(quat))
#trans = np.array(trans)
#trans = trans[::-1]
#times = np.array(times)
#times = times[::-1]
#rotations = np.array(rotations)    
#rotations = rotations[::-1]
#vec = np.quaternion(0, 0, 0, -1)
#quivers = (rotations*vec*rotations.conjugate())
#directions = quaternion.as_float_array(quivers)[:, 1:]
parser = argparse.ArgumentParser(description="Read tf_footprint csv file of a with certain bagfile and config")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")
parser.add_argument('config', action = 'store', 
		    help = "config file")
args = parser.parse_args()

file = '/home/joel/bagfiles/record_files_' + args.bagfile + '/tf_' + args.bagfile + '_footprint_' + args.config + '.csv'

#file = '/home/joel/bagfiles/record_files/tf_footprint.csv'

trans = []
times = []
rotations = []
with open(file) as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		trans.append(list(map(float,row[1:3])))
		#print(list(map(float,row[1:3])))		
		times.append(float(row[0])) 	
		quat = np.array(list(map(float, row[4:9])))[[0, 1, 2, 3]]
		rotations.append(quaternion.from_float_array(quat))

trans = np.array(trans)
#trans = trans[::-1]
times = np.array(times)
#times = times[::-1]
rotations = np.array(rotations)
#rotations = rotations[::-1]
vec = np.quaternion(0,0,0,-1)
quivers = (rotations*vec*rotations.conjugate())
directions = quaternion.as_float_array(quivers)[:,1:]

# %%
# Select the circles
# both circs = 80, 25*20, with -1 up top
plt.figure()
selection = np.arange(1,260)

plt.plot(trans[selection,0], trans[selection,1], '-')
plt.quiver(trans[selection,0], trans[selection,1], \
           directions[selection,0], directions[selection,1])

ax = plt.gca()
ax.set_aspect('equal')
for i, xy in enumerate(zip(trans[selection[::20], 0], trans[selection[::20], 1])):
    ax.annotate(str(i), xy)
# %% 
# plot circles measurement and solve for center and radius    
plt.figure()
circ = []
circ_times = []
circ.append(range(1,260))

center = []
radius = []
for c in circ:
    plt.plot(trans[c, 0], trans[c, 1], '-')
    x = trans[c, 0]
    y = trans[c, 1]
    A = np.column_stack((-2*x, -2*y, np.ones((len(x), 1))))
    b = (x**2 + y**2)
    sol = (np.linalg.lstsq(A, -b)[0])
    center.append(sol[:2])
    radius.append(np.sqrt(-sol[2] + sol[0]**2 + sol[1]**2))
    
center = np.array(center)
radius = np.array(radius)
# %%
# plot circles based on solved values for center and radius
plt.figure()
circ_plot = []
for i, c in enumerate(circ):
    plt.plot(trans[c, 0], trans[c, 1])
    circ_plot.append(plt.Circle(center[i], radius[i], color='r', fill=False))
for c_plot in circ_plot:
    plt.gca().add_artist(c_plot)
plt.plot(center[:, 0], center[:, 1], 'x')
plt.gca().set_aspect('equal')
plt.title('Radius = %0.3f m'%radius[0])

# %% Calculate the errors 
angles = np.array([np.arccos((trans[c.start, :2]-cent).dot(trans[c.stop-1, :2]-cent)\
                  /np.linalg.norm(trans[c.start, :2]-cent)\
                  /np.linalg.norm(trans[c.stop-1, :2]-cent))\
                    for c, cent in zip(circ, center)]) #angle between start and stop of input data
start_angles = np.array([np.arccos((trans[c.start, :2]-cent).dot(np.array([1, 0]))\
                  /np.linalg.norm(trans[c.start, :2]-cent)) \
                    for c, cent in zip(circ, center)]) #angle between positive x axis and start angle

# tune this to take the correct value of arccos:   
angles =  np.array([angles[0]])

# tune this to take the correct value of arccos:  
start_angles = 2*np.pi - np.array([start_angles[0]])

# tune this to account for number of turns:    
rad_per_point =  2*np.pi*np.array([18 - angles[0]/2/np.pi])/ \
            np.array(list(map(len, circ)))
# tune this again:
total_rad = 2*np.pi*np.array([18 - angles[0]/2/np.pi])
print(total_rad)

circ_angles = [(times[rng]-times[rng.start])/(times[rng.stop-1] - times[rng.start])*tr + sa \
               for rng, tr, sa in zip(circ, total_rad, start_angles)]
circ_angles_old = (list(map(np.arange, list(map(len, circ)))))*rad_per_point + start_angles

sim_points = [cent + np.column_stack((rad*np.cos(circ_a), rad*np.sin(circ_a))) \
                  for rad, circ_a, cent in zip(radius, circ_angles, center)]
sim_points_old = [cent + np.column_stack((rad*np.cos(circ_a), rad*np.sin(circ_a))) \
                  for rad, circ_a, cent in zip(radius, circ_angles_old, center)]

#sim_points = sim_points.reverse()
#print(sim_points[0])
#print(trans[80:204])
sim_points_2 = sim_points[0][::-1]
sim_points_2_old = sim_points[0][::-1]
#print(sim_points_2[:,1])
#print(len(sim_points[0]))
error = [np.linalg.norm(sim_p - trans[selection], axis=1) for sim_p, c in zip(sim_points, circ)]
error_xy = [ np.abs(sim_p - trans[selection]) for sim_p, c in zip(sim_points, circ)]
all_error = np.hstack(error)
all_error_xy = np.hstack(error_xy)
error_old = [np.linalg.norm(sim_p - trans[c, :2], axis=1) for sim_p, c in zip(sim_points_old, circ)]
all_error_old = np.hstack(error_old)

# %% plot CDF for norm error
plt.figure()
for i in range(len(circ)):
    plt.subplot(1,len(circ),i+1)
    plt.hist(error[i], 50, normed=1, histtype='step', cumulative=True)
    plt.title('CDF of errors with median error = %0.3f' % (np.median(error[i])))
#plt.suptitle('CDF of errors across all three circles, with median error = %0.3f' % (np.median(all_error)))
# %% plot CDF for XY error
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

# %% plot the simulation points and the actual translation measurements 
plt.figure()
limit = 50
for i, c in enumerate(circ):
    plt.plot(sim_points[i][:limit, 0], sim_points[i][:limit, 1], '.')    
    plt.plot(trans[c.start:c.start+limit, 0], trans[c.start:c.start+limit, 1], 'x')

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

plt.figure()
plt.hist(two_error, 50, density=1, histtype='step', cumulative=True, \
         linewidth=3)
plt.title('CDF of errors with median error = %0.3f' % (np.median(two_error)))

prop = ['bar', False]
plt.figure()
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
# %%
np.savetxt('circ_error.txt', all_error)
# %%
plt.figure()
directions_tangents = np.array([-np.sin(circ_angles), np.cos(circ_angles)])[:, 0, :].T
plt.plot(sim_points[0][:, 0], sim_points[0][:, 1], 'k-')
plt.plot(trans[selection, 0], trans[selection, 1], '-')
plt.quiver(trans[selection, 0], trans[selection, 1], \
           directions[selection, 0], directions[selection, 1], color='r')
plt.quiver(sim_points[0][:, 0], sim_points[0][:, 1], \
           directions_tangents[:, 0], directions_tangents[:, 1], color='b')
#plt.quiver(sim_points[0][:, 0], sim_points[0][:, 1], \
#           directions[selection, 0], directions[selection, 1], color='r')
# %%
plt.figure()
directions_diff = np.arccos(np.einsum('ij,ij->i', \
                          directions_tangents, directions[selection, 0:2]) \
                    /(np.linalg.norm(directions[selection, 0:2], axis=1)))
directions_diff_deg = directions_diff*180/np.pi
plt.hist(directions_diff, 50, normed=1, histtype='step', cumulative=True)
plt.title('CDF of angle errors with median error = %0.3f' % (np.median(directions_diff)))
# %%
np.savetxt('angle_errors.txt', directions_diff_deg)
#%%
arr = np.einsum('ij,ij->i', directions_tangents, directions[selection, 0:2])
#plt.show()
plt.figure()
sim_points = np.array(sim_points)
plt.plot(range(1,260),(sim_points[0][0:260,0]))
plt.plot(range(1,260),(trans[1:260,0]))
plt.figure()
plt.plot(range(1,260),(sim_points[0][0:260,1]))
plt.plot(range(1,260),(trans[1:260,1]))

plt.figure()
plt.plot(sim_points[0][212:238,0], sim_points[0][212:238,1])
#plt.plot(sim_points[0][137:141,0], sim_points[0][137:141,1], '.')
#print(np.sqrt((sim_points[0][137:141,:] - center)**2))
plt.figure()
#plt.plot(sim_points[0][0:25,0], sim_points[0][0:25,1])
#plt.figure()
#plt.plot(sim_points[0][26:51,0], sim_points[0][26:51,1])
plt.show()
#print(circ)
