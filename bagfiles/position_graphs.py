import rosbag
import argparse
import os.path
import os
import inspect
import numpy as np
import csv
import matplotlib.pyplot as plt
import pdb

#input arguments
parser = argparse.ArgumentParser(description="Plots trajectories of model states (ground truth), odom,robot pose ekf, and cartographer trajectories")
parser.add_argument('-i', '--input', required = True,
		    help = "input bag file")
args = parser.parse_args()


#open bag file and create directory for new files
bag = rosbag.Bag(args.input + '.bag')
#path = args.input + "_topics"
#os.mkdir(path)
#os.chdir(path)
inputfile = "/home/joel/bagfiles/" + args.input + "/" + args.input + "_output.csv"

inputcsv = open(inputfile, 'wb')
writer = csv.writer(inputcsv, delimiter=',')

#Collect model states and create csv file
modelstates = []
row = ['model states x', 'model states y']
writer.writerow(row)
for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
	#prints raw message
	print(msg)
	try:
		robot_index = msg.name.index('mobile_base_with_yd')	
		#adds gazebo x and y data to csv	
		modelstates.append([msg.pose[robot_index].position.x, msg.pose[robot_index].position.y])
		row = []
		row.extend([msg.pose[robot_index].position.x, msg.pose[robot_index].position.y])
		writer.writerow(row)
	except:
		print('No mobile_base')
#Collect odometry messages and create csv file
odom = []
row = ['odom x', 'odom y']
writer.writerow(row)
for topic, msg, t in bag.read_messages(topics=['/odom']):
	#prints raw message	
	print(msg)
	#adds odometry position to csv	
	odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
	row = []
	row.extend([msg.pose.pose.position.x, msg.pose.pose.position.y])
	writer.writerow(row)

#Collect odom_combined messages and create csv file
robot_pose_ekf = []
row = ['robot_pose x', 'robot_pose y']
writer.writerow(row)
for topic, msg, t in bag.read_messages(topics=['/robot_pose_ekf/odom_combined']):
	#prints raw message	
	print(msg)
	#adds odometry position to csv	
	robot_pose_ekf.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
	row = []
	row.extend([msg.pose.pose.position.x, msg.pose.pose.position.y])
	writer.writerow(row)
#pdb.set_trace()

#Cartographer trajectories
combined_trajectories = []
combinedfile = "/home/joel/bagfiles/" + args.input + "/tq_" + args.input + "_combined.csv"
combinedcsv = open(combinedfile, 'r')
reader = csv.reader(combinedcsv, delimiter=',')
row = ['combined trajectories x', 'combined trajectories y']
writer.writerow(row)
for entry in reader:
	#adds odometry position to csv	
	combined_trajectories.append([entry[1], entry[2]])
	row = []
	row.extend([entry[1], entry[2]])
	writer.writerow(row)


imu_only_trajectories = []
imu_only_file = "/home/joel/bagfiles/" + args.input + "/tq_" + args.input + "_imu_only.csv"
imu_only_csv = open(imu_only_file, 'r')
reader = csv.reader(imu_only_csv, delimiter=',')
row = ['imu_only trajectories x', 'imu_only trajectories y']
writer.writerow(row)
for entry in reader:
	#adds odometry position to csv	
	imu_only_trajectories.append([entry[1], entry[2]])
	row = []
	row.extend([entry[1], entry[2]])
	writer.writerow(row)


lidar_only_trajectories = []
lidar_only_file = "/home/joel/bagfiles/" + args.input + "/tq_" + args.input + "_lidar_only.csv"
lidar_only_csv = open(lidar_only_file, 'r')
reader = csv.reader(lidar_only_csv, delimiter=',')
row = ['lidar_only trajectories x', 'lidar_only trajectories y']
writer.writerow(row)
for entry in reader:
	#adds odometry position to csv	
	lidar_only_trajectories.append([entry[1], entry[2]])
	row = []
	row.extend([entry[1], entry[2]])
	writer.writerow(row)

modelstates1 = np.array(modelstates)
odom1 = np.array(odom)
robot_pose_ekf1 = np.array(robot_pose_ekf)
combined_trajectories1 = np.array(combined_trajectories)
imu_only_trajectories1 = np.array(imu_only_trajectories)
lidar_only_trajectories1 = np.array(lidar_only_trajectories)
#pdb.set_trace()


plt.figure()
plt.plot(modelstates1[:,0], modelstates1[:,1], color = 'g',marker = '^', label = '/gazebo/model_states')
plt.plot(odom1[:,0], odom1[:,1], color = 'y', marker = '^', label = '/odom')
plt.plot(robot_pose_ekf1[:,0], robot_pose_ekf1[:,1], marker = '^', color = 'r', label = '/robot_pose_ekf')
plt.plot(combined_trajectories1[:,0], combined_trajectories1[:,1], marker = '^', color = 'b', label = 'combined trajectory')
plt.plot(imu_only_trajectories1[:,0], imu_only_trajectories1[:,1], marker = '^', color = 'c', label = 'imu_only trajectory')
plt.plot(lidar_only_trajectories1[:,0], lidar_only_trajectoriess1[:,1], marker = '^', color = 'k', label = 'lidar_only trajectory')
plt.legend(loc="upper left")
plt.show()
