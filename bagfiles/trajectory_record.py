import csv
import argparse
import numpy as np
import os
import subprocess
import pdb

parser = argparse.ArgumentParser(description="Read tf_footprint csv file of a with certain bagfile and config")
parser.add_argument('bagfile', action = "store", type = str,
		    help = "input bag file")

args = parser.parse_args()

base = args.bagfile

inputcsv = "/home/joel/bagfiles/" + base + "/input.csv"

csv_reader = csv.reader(open(inputcsv, 'r'))
#lines = list(csv_reader)
#outputcsv = "/home/joel/bagfiles/"+ base + "/output.csv"
#csv_writer = csv.writer(open(outputcsv, 'w+'))
#csv_writer.writerows(lines)

for row in csv_reader:
	
	arg = ['. ~/kobuki_sim/bagfiles/straight_line_record.sh $1 $2 $3', '1' , args.bagfile,row[0],row[1]]
	#pdb.set_trace()	
	subprocess.call(['echo $1 $2 $3 $4', '1', args.bagfile, row[0], row[1]], shell = True)	
	subprocess.call(arg, shell = True)
	
	#subprocess.call(["python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 $4", '1', args.bagfile, row[0], row[1], 'combined'], shell = True)
	#subprocess.call(["python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 $4", '1', args.bagfile, row[0], row[1], 'imu_only'], shell = True)
	#subprocess.call(["python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 $4", '1', args.bagfile, row[0], row[1], 'lidar_only'], shell = True)

subprocess.call(["python ~/kobuki_sim/bagfiles/plot_error.py $1", '1', "output.csv"], shell = True)
