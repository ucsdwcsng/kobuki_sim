import re
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize as opt 
#import quaternion
import csv
import argparse
import rosbag


parser = argparse.ArgumentParser(description="Export odom topic (ground truth) to csv")
parser.add_argument('bagfile', action = "store",
		    help = "input bag file")


args = parser.parse_args()
base = args.bagfile

bag = rosbag.Bag("/home/joel/bagfiles/" + args.bagfile + "/" + base + ".bag")

#get ground truth from odom
odomcsv = open("groundTruth_" + base + ".csv", 'wb')
writer = csv.writer(odomcsv, delimiter=',')
for topic, msg, t in bag.read_messages(topics=['/joint_states']):
	row = []	
	row.extend([msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs,msg.position[0], msg.position[1]])
	writer.writerow(row)	
#for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
#	row = []	
#	row.extend([msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs,msg.pose[11].position.x, msg.pose[11].position.y])
#	writer.writerow(row)	

print("Saved Ground Truth: groundTruth_" + base + ".csv")
