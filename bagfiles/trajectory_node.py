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
parser = argparse.ArgumentParser(description="Export cartographer trajectory bag file (generated from pbstream) to a csv file")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")
parser.add_argument('imu_noise', action='store', help = 'imu noise stddev')
parser.add_argument('lidar_noise', action='store', help = 'lidar noise stddev')
parser.add_argument('config', action = 'store', 
		    help = "config file")

args = parser.parse_args()

base = args.bagfile + '_imu-' + args.imu_noise + '_lidar-' + args.lidar_noise + '_' + args.config

bag = rosbag.Bag('/home/joel/bagfiles/' + args.bagfile + '/tq_' + base + '.bag')

messages = [msgs for topics, msgs, time in bag.read_messages(['trajectory_0'])]

time_positions = [[msgs.header.stamp.secs + 1e-9*msgs.header.stamp.nsecs, msgs.transform.translation.x, msgs.transform.translation.y] for msgs in messages]

inputcsv = '/home/joel/bagfiles/' + args.bagfile + '/tq_' + base + '.csv'

csv_writer = csv.writer(open(inputcsv, 'w+'))

for entry in time_positions:
	row = []
	row.extend([entry[0], entry[1], entry[2]])
	csv_writer.writerow(row)

print("Saved Trajectory for " + args.config + ":tq_" + base + ".csv")
