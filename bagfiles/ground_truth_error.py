import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize as opt 
#import quaternion
import csv
import argparse
import rosbag
import pdb

parser = argparse.ArgumentParser(description="Measures error of cartographer trajectory against ground truth (odom topic)")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")

parser.add_argument('imu_noise', action = "store",
		    help = "imu noise standard deviation")
parser.add_argument('lidar_noise', action = 'store', 
		    help = "lidar noise standard deviation")

parser.add_argument('config', action = 'store', 
		    help = "config file")
args = parser.parse_args()

base = args.bagfile + "_imu-" + args.imu_noise + "_lidar-" + args.lidar_noise

tq = "/home/joel/bagfiles/" + args.bagfile + "/tq_" + base + "_" + args.config + ".csv"
groundTruth = "/home/joel/bagfiles/" + args.bagfile + "/groundTruth_" + base + ".csv"

#get ground truth from odom
odom = []
with open(groundTruth) as f:
	coordinates = csv.reader(f)
	odom = {float(row[0]): ([float(row[1]), float(row[2])]) for row in coordinates}

#cartographer trajectory
#trajectory = []
with open(tq) as f:
	coordinates = csv.reader(f)
	trajectory = {float(row[1])+1e-9*float(row[2]):[float(row[3]), float(row[4])] for row in coordinates}

truth = {keys:values for keys, values in odom.items() if (keys in trajectory)}


#plot ground truth and trajectory
axs = plt.subplots(1,2)[1]
for time,[x,y] in truth.items():
	axs[0].plot(time,x, 'o', label = 'truth')
	axs[1].plot(time,y, 'o', label = 'truth')

for time,[x,y] in trajectory.items():
	axs[0].plot(time,x, marker = 'x', label = 'truth')
	axs[1].plot(time,y, marker = 'x', label = 'truth')
#plt.gca().set_aspect('equal')
fig = plt.figure()
for time, [x,y] in truth.items():
	plt.plot(x,y, marker = 'o')
	plt.plot(trajectory.get(time)[0], trajectory.get(time)[1], marker = 'x')
fig.savefig(base + '_' + args.config + '_trajectory.png')
#get errors

errorx = [abs(trajectory[(keys)][0] - values[0]) for keys,values in truth.items() if keys in trajectory]
errorx = errorx - np.min(errorx)
errory = [abs(trajectory[(keys)][1] - values[1]) for keys,values in truth.items() if keys in trajectory]
errory = errory - np.min(errory)	

errorxy = [list([errorx[i], errory[i]]) for i in range(0, len(errorx)-1)]
error = [np.linalg.norm(coordinates) for coordinates in errorxy]
errorxy = np.array(errorxy)

#pdb.set_trace()
error_csv = "/home/joel/bagfiles/" + args.bagfile + "/" + base + '_' + args.config + '_error.csv'
print(error_csv)
with open(error_csv, 'w') as f:
	writer = csv.writer(f)
	for entry in error:
		writer.writerow([entry])

#pdb.set_trace()
#%%
#Plot of Cumulative Distribution of Norm Error
circ = []
circ.append(range(0,len(trajectory.values())))
fig = plt.figure()
for i in range(len(circ)):
    plt.subplot(1,len(circ),i+1)
    plt.hist(error, 50, normed = 1, histtype = 'step', cumulative = True)
    plt.title('CDF of errors with median error = %0.6f' %(np.median(error)))
#%%
fig.savefig(base + '_' + args.config +  '_overall_error.png')
for i in range(len(circ)):
    fig = plt.figure()
    plt.subplot(121)
    plt.hist(errorxy[:, 0], 50, normed=1, histtype='step', cumulative=True)
    plt.title('X-axis errors median error = %0.6f' % \
              (np.median(errorxy[:, 0])))   
    
    plt.subplot(122)
    plt.hist(errorxy[:, 1], 50, normed=1, histtype='step', cumulative=True)
    plt.title('Y-axis median error = %0.6f' % \
              (np.median(errorxy[:, 1])))
plt.suptitle('CDF of X & Y Errors')  
fig.savefig(base + '_' + args.config + '_xy_error.png')
#plt.show()

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
