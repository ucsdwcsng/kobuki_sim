import argparse
import os.path
import os
import inspect
import numpy as np
import csv
import matplotlib.pyplot as plt








#graph csv plots and compare

odomx = []
odomy = []
with open('odomData.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		odomx.append(float(row[0]))
		odomy.append(float(row[1]))

plt.subplot(2,3,1)
plt.plot(odomx, odomy, marker='o')
plt.plot(odomx[0],odomy[0], marker = '^')
plt.axis('equal') 
plt.plot(odomx[len(odomx)-1], odomy[len(odomy)-1], marker = 'v')
plt.title('Position based on \odom')

odom_combinedx = []
odom_combinedy = []
with open('odom_combinedData.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		odom_combinedx.append(float(row[0]))
		odom_combinedy.append(float(row[1]))

plt.subplot(2,3,2)
plt.plot(odom_combinedx, odom_combinedy, marker='o')
plt.plot(odom_combinedx[0],odom_combinedy[0], marker = '^')
plt.axis('equal') 
plt.plot(odom_combinedx[len(odom_combinedx)-1], odom_combinedy[len(odom_combinedy)-1], marker = 'v')
plt.title('Position based on \\robot_pose_ekf\odom_combined')


modelx = []
modely = []
with open('modelStates.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		modelx.append(float(row[0]))
		modely.append(float(row[1]))


plt.subplot(2,3,3)
plt.plot(modelx, modely, marker='o')
plt.plot(modelx[0],modely[0], marker = '^')
plt.axis('equal')
plt.plot(modelx[len(modelx)-1], modely[len(modely)-1], marker = 'v') 
plt.title('Position based on \gazebo\model_states')
	 
#check position calculations for imu data
time = []
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

time.insert(0, (2*time[0]-time[1])) 

for i in range (1, len(time)-1):
	velx.append(velx[i-1] + accelx[i] * (time[i]-time[i-1]))
	posx.append(posx[i-1] + velx[i] * (time[i]-time[i-1]))
	vely.append(vely[i-1] + accely[i] * (time[i]-time[i-1]))
 	posy.append(posy[i-1] + vely[i] * (time[i]-time[i-1]))

plt.subplot(2,3,4)
plt.plot(posx, posy, marker='o')
plt.axis('equal')
plt.plot(posx[0],posy[0], marker = '^')
plt.plot(posx[len(posx)-1], posy[len(posy)-1], marker = 'v') 
plt.title('Position based on \mobile_base\sensors\imu_data')


##check how to collect tf data

linky = []
linkx = []
with open('tf_link.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		linkx.append(float(row[1]))
		linky.append(float(row[2]))


plt.subplot(2,3,5)
plt.plot(linkx, linky, marker='o')
plt.plot(linkx[0],linky[0], marker = '^')
plt.axis('equal')
plt.plot(linkx[len(linkx)-1], linky[len(linky)-1], marker = 'v') 
plt.title('Position based on \\tf echo \\map \\base_link')


footprintx = []
footprinty = []
with open('tf_footprint.csv') as f:
	coordinates = csv.reader(f)
	for row in coordinates:
		footprintx.append(float(row[1]))
		footprinty.append(float(row[2]))


plt.subplot(2,3,6)
plt.plot(footprintx, footprinty, marker='o')
plt.plot(footprintx[0],footprinty[0], marker = '^')
plt.axis('equal')
plt.plot(footprintx[len(footprintx)-1], footprinty[len(footprinty)-1], marker = 'v') 
plt.title('Position based on \\tf echo \\map \\base_footprint')
plt.show()
