import rosbag
import argparse
import os.path
import os
import inspect
import numpy as np
import csv
import matplotlib.pyplot as plt


#input arguments
parser = argparse.ArgumentParser(description="Read topics from bagfiles then graph imu and odom data to compare")
parser.add_argument('-i', '--input', required = True,
		    help = "input bag file")
args = parser.parse_args()


#open bag file and create directory for new files
bag = rosbag.Bag(args.input)
#path = args.input + "_topics"
#os.mkdir(path)
#os.chdir(path)


#Collect odometry messages and create csv file
odomcsv = open("odomData.csv", 'wb')
writer = csv.writer(odomcsv, delimiter=',')

odomText = open("odomMessages.txt", "w+")

for topic, msg, t in bag.read_messages(topics=['/odom']):
	#prints raw message	
	odomText.write(str(msg) + '\n\n')
	print(msg)
	#adds odometry position to csv
	row = []	
	row.extend([msg.pose.pose.position.x, msg.pose.pose.position.y])
	writer.writerow(row)

odomText.close()	
odomcsv.close()


#Collect mobile base imu data and create csv file
mobilebasecsv = open("imuData.csv", 'wb')
writer = csv.writer(mobilebasecsv, delimiter = ',')

mobile_base_imu_Test = open("mobile_base_imu_Messages.txt", "w+")

for topic, msg, t in bag.read_messages(topics=['/mobile_base/sensors/imu_data']):
	#prints raw message
	mobile_base_imu_Test.write(str(msg) + "\n\n")
	print(msg)	
	#adds imu linear acceleration (x and y) to csv
	row = []	
	row.extend([msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs, msg.linear_acceleration.x, msg.linear_acceleration.y])
	writer.writerow(row)
mobilebasecsv.close()
mobile_base_imu_Test.close()



#Collect model states and create csv file
modelstatescsv = open('modelStates.csv', 'wb')
writer = csv.writer(modelstatescsv, delimiter = ',')

model_states_Test = open("model_states_Messages.txt", "w+")

for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
	#prints raw message
	model_states_Test.write(str(msg) + "\n\n")
	print(msg)	
	#adds gazebo x and y data to csv	
	row = []
	row.extend([msg.pose[11].position.x, msg.pose[11].position.y])
	writer.writerow(row)
modelstatescsv.close()
model_states_Test.close()



#rosbag does not save all tf transforms
#rosrun tf_echo map base_link, map base_footprint
##tfecholinkcsv = open('tfecholink.csv', 'wb')
##writerlink = csv.writer(tfecholinkcsv, delimiter = ',')

##tfechofootprintcsv = open('tfechofootprint.csv', 'wb')
##writerfootprint = csv.writer(tfechofootprintcsv, delimiter = ',')

##tf_Test = open("tf_Messages.txt", "w+")

##for topic, msg, t in bag.read_messages(topics=['/tf']):
##	#prints raw message
##	tf_Test.write(str(msg) + "\n\n")
##	print(msg)	
##	#adds gazebo x and y data to csv	
##	for entry in msg.transforms:
##		if entry.header.frame_id == 'map':
##			row = [entry.transform.translation.x, entry.transform.translation.y]
##			if entry.child_frame_id =='base_link':
##				writerlink.write(row)
##			if entry.child_frame_id == 'base_footprint':
##				writerfootprint.write(row)
##
##tfecholinkcsv.close()
##tfechofootprintcsv.close()
##tf_Test.close()




bag.close()


