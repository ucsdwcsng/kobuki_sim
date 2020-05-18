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
path = args.input + "_topics"
os.mkdir(path)
os.chdir(path)


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

#rosrun tf_echo map base_link, map base_footprint

bag.close()

#graph csv plots and compare

odomx = []
odomy = []
with open('odomData.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		odomx.append(float(row[0]))
		odomy.append(float(row[1]))

plt.subplot(1,3,1)
plt.plot(odomx, odomy, marker='o')
plt.plot(odomx[0],odomy[0], marker = '^')
 
plt.plot(odomx[len(odomx)-1], odomy[len(odomy)-1], marker = 'v')
plt.title('Position based on \odom')


modelx = []
modely = []
with open('modelStates.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		modelx.append(float(row[0]))
		modely.append(float(row[1]))


plt.subplot(1,3,2)
plt.plot(modelx, modely, marker='o')
plt.plot(modelx[0],modely[0], marker = '^')
plt.plot(modelx[len(modelx)-1], modely[len(modely)-1], marker = 'v') 
plt.title('Position based on \gazebo\model_states')
	 

time = [31486]
accelx = [0]
accely = [0]
velx = [0]
vely = [0]
posx = [0]
posy = [0]
with open('imuData.csv') as f:
	acceleration = csv.reader(f)
	for row in acceleration:
		time.append(float(row[0]))
		accelx.append(float(row[1]))
		accely.append(float(row[2]))

for i in range (1, len(time)-1):
	velx.append(velx[i-1] + accelx[i] * (time[i]-time[i-1]))
	posx.append(posx[i-1] + velx[i] * (time[i]-time[i-1]))
	vely.append(vely[i-1] + accely[i] * (time[i]-time[i-1]))
 	posy.append(posy[i-1] + vely[i] * (time[i]-time[i-1]))

plt.subplot(1,3,3)
plt.plot(posx, posy, marker='o')
plt.plot(posx[0],posy[0], marker = '^')
plt.plot(posx[len(posx)-1], posy[len(posy)-1], marker = 'v') 
plt.title('Position based on \mobile_base\sensors\imu_data')
plt.show()	


