import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize as opt 
#import quaternion
import csv
import argparse
import rosbag


parser = argparse.ArgumentParser(description="Read tf_footprint csv file of a with certain bagfile and config")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")

parser.add_argument('imu_noise', action = "store",
		    help = "input bag file")
parser.add_argument('lidar_noise', action = 'store', 
		    help = "config file")
args = parser.parse_args()
base = args.bagfile + "_imu-" + args.imu_noise + "_lidar-" + args.lidar_noise

bag = rosbag.Bag("/home/joel/bagfiles/" + args.bagfile + "/" + base + ".bag")

#get ground truth from odom
odomcsv = open("groundTruth_" + base + ".csv", 'wb')
writer = csv.writer(odomcsv, delimiter=',')
for topic, msg, t in bag.read_messages(topics=['/odom']):
	row = []	
	row.extend([msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs,msg.pose.pose.position.x, msg.pose.pose.position.y])
	writer.writerow(row)	
#for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
#	row = []	
#	row.extend([msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs,msg.pose[11].position.x, msg.pose[11].position.y])
#	writer.writerow(row)	

print("Saved Ground Truth")
