import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt

import subprocess
import pdb

parser = argparse.ArgumentParser(description="Read tf_footprint csv file of a with certain bagfile and config")
parser.add_argument('input', action = "store",
		    help = "input bag file")
args = parser.parse_args()
inputcsv = args.input

imu_noise_value = []
imu_noise_combined_error = []
imu_noise_imu_error = []
imu_noise_lidar_error = []

lidar_noise_value = []
lidar_noise_combined_error = []
lidar_noise_imu_error = []
lidar_noise_lidar_error = []


csv_reader = csv.reader(open(inputcsv, 'r'))

for row in csv_reader:
	if row[0] != 'default':
		imu_noise_value.append(row[0])
		imu_noise_combined_error.append(row[2])
		imu_noise_imu_error.append(row[3])
		imu_noise_lidar_error.append(row[4])
	if row[1] != 'default':
		lidar_noise_value.append(row[1])
		lidar_noise_combined_error.append(row[2])
		lidar_noise_imu_error.append(row[3])
		lidar_noise_lidar_error.append(row[4])

plt.figure()

plt.subplot(1,2,1)
plt.plot(imu_noise_value, imu_noise_combined_error, marker = 'o', label = 'combined')
plt.plot(imu_noise_value, imu_noise_imu_error, marker = 'x', label = 'imu_error')
plt.plot(imu_noise_value, imu_noise_lidar_error, marker = '*', label = 'lidar_error')
plt.title("IMU Noise Errors")
plt.xlabel("IMU Noise StdDev (m)")
plt.ylabel("Error value (m)")
plt.legend()


plt.subplot(1,2,2)
plt.plot(lidar_noise_value, lidar_noise_combined_error, marker = 'o', label = 'combined')
plt.plot(lidar_noise_value, lidar_noise_imu_error, marker = 'x', label = 'imu_error')
plt.plot(lidar_noise_value, lidar_noise_lidar_error, marker = '*', label = 'lidar_error')
plt.legend()
plt.title("Lidar Noise Errors")
plt.xlabel("Lidar Noise StdDev (m)")
plt.ylabel("Error value (m)")

plt.suptitle(args.input + " Error")
plt.show()
