import argparse
import csv
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
import pdb
import seaborn as sns

parser = argparse.ArgumentParser(description="Outputs median error for each time stamp to measure drift over long periods of time")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")

args = parser.parse_args()

csv_file = '/home/joel/bagfiles/' + args.bagfile + '/output.csv'

csv_reader = csv.reader(open(csv_file, 'r'))

noise_list = []
noise_list_string = []
for row in csv_reader:
	noise_list.append([row[0], row[1]])
	noise_list_string.append("imu:" + row[0] + ", lidar:" + row[1]) 
selection_combined = 50
selection_imu_only = 400
selection_lidar_only = 50
error_matrix_combined = [[0 for x in range(selection_combined)] for y in range(60)]

error_matrix_imu_only = [[0 for x in range(selection_imu_only)] for y in range(60)]

error_matrix_lidar_only = [[0 for x in range(selection_lidar_only)] for y in range(60)]


	
for i in range(60):
	
	error_combined_csv = "/home/joel/bagfiles/" + args.bagfile + "/" + args.bagfile + "_imu-" + noise_list[i][0] + "_lidar-" + noise_list[i][1] + "_combined_error.csv"	
	data = [row for row in csv.reader(open(error_combined_csv,'r'))]	
	#pdb.set_trace()	
	for k in range(selection_combined):
		#pdb.set_trace()		
		error_matrix_combined[i][k] = float(data[k][0])
				
	error_imu_only_csv = "/home/joel/bagfiles/" + args.bagfile + "/" + args.bagfile + "_imu-" + noise_list[i][0] + "_lidar-" + noise_list[i][1] + "_imu_only_error.csv"	
	data = [row for row in csv.reader(open(error_imu_only_csv,'r'))]
	for k in range(selection_imu_only):
		error_matrix_imu_only[i][k] = float(data[k][0])

	error_lidar_only_csv = "/home/joel/bagfiles/" + args.bagfile + "/" + args.bagfile + "_imu-" + noise_list[i][0] + "_lidar-" + noise_list[i][1] + "_lidar_only_error.csv"	
	data = [row for row in csv.reader(open(error_lidar_only_csv,'r'))]
	for k in range(selection_lidar_only):
		error_matrix_lidar_only[i][k] = float(data[k][0])
#pdb.set_trace()

plt.subplot(1,3,1)	
ax = sns.heatmap(error_matrix_combined, robust = True, linewidth = 0.5, cmap = "YlGnBu", vmax = 0.5)
plt.title('Combined Error')
plt.xlabel('Time')
plt.ylabel('Noise Parameters')

plt.subplot(1,3,2)	
ax = sns.heatmap(error_matrix_imu_only, linewidth = 0.5, cmap = "YlGnBu", vmax = 0.5)
plt.title('IMU Error')
plt.xlabel('Time')
plt.ylabel('Noise Parameters')

plt.subplot(1,3,3)	
ax = sns.heatmap(error_matrix_lidar_only, linewidth = 0.5, cmap = "YlGnBu", vmax = 0.5)
plt.title('Lidar Error')
plt.xlabel('Time')
plt.ylabel('Noise Parameters')

plt.show()
