rosrun tf tf_echo /map /base_footprint 10 > tf_$1_footprint_$2.txt &
roslaunch kobuki_yd cartographer_bag_record.launch config:=$2 bag_filenames:=$1

python ~/kobuki_sim/bagfiles/tf_parse_trajectory.py tf_$1_footprint_$2.txt

python3 ~/kobuki_sim/bagfiles/newtrajectory.py $1 $2
