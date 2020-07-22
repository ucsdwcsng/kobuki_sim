#!/usr/bin/env python3
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
#import quaternion
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

#file = '/home/joel/bagfiles/record_files_' + args.bagfile + '/tf_' + args.bagfile + '_footprint_' + args.config + '.csv'

file = '/home/joel/bagfiles/record_files_' + args.bagfile + '/tq_' + args.bagfile + '_' + args.config + '.csv'

trans = []
times = []
rotations = []
with open(file) as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		trans.append(list(map(float,row[3:5])))
		#print(list(map(float,row[1:3])))		
		times.append(float(row[1]) + 1e-9*float(row[2])) 	
#		quat = np.array(list(map(float, row[4:9])))[[0, 1, 2, 3]]
#		rotations.append(quaternion.from_float_array(quat))

trans = np.array(trans)
#trans = trans[::-1]
times = np.array(times)
#times = times[::-1]
#rotations = np.array(rotations)
#rotations = rotations[::-1]
#vec = np.quaternion(0,0,0,-1)
#quivers = (rotations*vec*rotations.conjugate())
#directions = quaternion.as_float_array(quivers)[:,1:]

# %%
# Select the circles
# both circs = 80, 25*20, with -1 up top
plt.figure()
selection = np.arange(1,950)

plt.plot(trans[selection,0], trans[selection,1], '-')
#plt.quiver(trans[selection,0], trans[selection,1], \
#           directions[selection,0], directions[selection,1])

ax = plt.gca()
ax.set_aspect('equal')
for i, xy in enumerate(zip(trans[selection[::20], 0], trans[selection[::20], 1])):
    ax.annotate(str(i), xy)
# %% 
# plot circles measurement and solve for center and radius    
plt.figure()
circ = []
circ_times = []
circ.append(range(selection[0],selection[len(selection)-1]))

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

diff = []
error = []
for c in circ:
    
    x = trans[c,0] - center[0][0]
    y = trans[c,1] - center[0][1]
    diff.append(np.column_stack((x,y)))
    

#plt.figure()
diff_normed = []
error_xy = []
for d in diff[0]:    
    error.append(abs(np.linalg.norm(d) - radius)[0])
    diff_normed.append(d/np.linalg.norm(d)*radius)
error_xy = (abs(diff[0] - diff_normed))
#plt.plot(diff_normed[:, 0], diff_normed[:,1])



#%%
#Plot of Cumulative Distribution of Norm Error
plt.figure()
for i in range(len(circ)):
    plt.subplot(1,len(circ),i+1)
    plt.hist(error, 50, normed = 1, histtype = 'step', cumulative = True)
    plt.title('CDF of errors with median error = %0.6f' %(np.median(error)))
#%%

for i in range(len(circ)):
    plt.figure()
    plt.subplot(121)
    plt.hist(error_xy[:, 0], 50, normed=1, histtype='bar', cumulative=True)
    plt.title('X-axis errors median error = %0.6f' % \
              (np.median(error_xy[:, 0])))   
    
    plt.subplot(122)
    plt.hist(error_xy[:, 1], 50, normed=1, histtype='bar', cumulative=True)
    plt.title('Y-axis median error = %0.6f' % \
              (np.median(error_xy[:, 1])))
plt.suptitle('CDF of X & Y Errors')  
plt.show()
