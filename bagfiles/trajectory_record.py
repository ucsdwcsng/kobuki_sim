import csv
import argparse
import numpy as np
import os
import subprocess
import pdb

parser = argparse.ArgumentParser(description="Run repeated simulations of gazebo simulation with various imu+lidar stddev then output cartographer trajectory errors")
parser.add_argument('bagfile', action = "store", type = str,
		    help = "input bag file")

args = parser.parse_args()

base = args.bagfile

inputcsv = "/home/joel/bagfiles/" + base + "/output.csv"

csv_reader = csv.reader(open(inputcsv, 'r'))
#lines = list(csv_reader)
#outputcsv = "/home/joel/bagfiles/"+ base + "/output.csv"
#csv_writer = csv.writer(open(outputcsv, 'w+'))
#csv_writer.writerows(lines)

for row in csv_reader:
	if (row[2] == '0' or row[3] == '0' or row[4] == '0'):
		arg = ['. ~/kobuki_sim/bagfiles/new_record.sh $1 $2 $3', '1' , args.bagfile,row[0],row[1]]
		#pdb.set_trace()	
		subprocess.call(['echo $1 $2 $3 $4', '1', args.bagfile, row[0], row[1]], shell = True)	
		subprocess.call(arg, shell = True)
	else:
		print("Skipped" + args.bagfile + " " + row[0] + " " + row[1])	


