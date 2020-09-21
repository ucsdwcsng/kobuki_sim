
#record gazebo simulation
roslaunch kobuki_yd record_straight_line.launch bagfile_name:=$1 imu_noise:=$2 lidar_noise:=$3
 

#run combined simulation
CONFIG=combined
#run cartographer w/ combined config and save pbstream
roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=$CONFIG
#convert pbstream to trajectory bag file
rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag --input $1_imu-$2_lidar-$3_$CONFIG.bag.pbstream --output tq_$1_imu-$2_lidar-$3_$CONFIG.bag
#convert trajectory bag file to csv of times & positions
python3 ~/kobuki_sim/bagfiles/trajectory_node.py $1 $2 $3 $CONFIG
#measure error using circle of best fit
python3 ~/kobuki_sim/bagfiles/trajectory.py $1 $2 $3 $CONFIG

#run imu_only simulation
CONFIG=imu_only
roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=$CONFIG
rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag --input $1_imu-$2_lidar-$3_$CONFIG.bag.pbstream --output tq_$1_imu-$2_lidar-$3_$CONFIG.bag
python3 ~/kobuki_sim/bagfiles/trajectory_node.py $1 $2 $3 $CONFIG
python3 ~/kobuki_sim/bagfiles/trajectory.py $1 $2 $3 $CONFIG

#run lidar_only simulation
CONFIG=lidar_only
roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=$CONFIG
rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag --input $1_imu-$2_lidar-$3_$CONFIG.bag.pbstream --output tq_$1_imu-$2_lidar-$3_$CONFIG.bag
python3 ~/kobuki_sim/bagfiles/trajectory_node.py $1 $2 $3 $CONFIG
python3 ~/kobuki_sim/bagfiles/trajectory.py $1 $2 $3 $CONFIG

sudo rm $1_imu-$2_lidar-$3.bag tq_$1_imu-$2_lidar-$3_combined.bag tq_$1_imu-$2_lidar-$3_imu_only.bag tq_$1_imu-$2_lidar-$3_lidar_only.bag
