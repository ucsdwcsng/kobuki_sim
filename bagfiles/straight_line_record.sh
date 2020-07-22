
#echo $1 $2 $3
#record bagfiles
timeout 5m roslaunch kobuki_yd record_straight_line.launch bagfile_name:=$1 imu_noise:=$2 lidar_noise:=$3 
python ~/kobuki_sim/bagfiles/odom_parse.py $1 $2 $3
touch tq_$1_imu-$2_lidar-$3_combined.txt
touch tq_$1_imu-$2_lidar-$3_imu_only.txt
touch tq_$1_imu-$2_lidar-$3_lidar_only.txt
wait

#run combined error
timeout 2m roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=combined  &
(sleep 30s && echo "Run trajectory query" && timeout 30s rosservice call /trajectory_query 0 > tq_$1_imu-$2_lidar-$3_combined.txt)
sleep 5s
python ~/kobuki_sim/bagfiles/tq_parse.py tq_$1_imu-$2_lidar-$3_combined.txt
python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 combined
wait

#run imu error
timeout 2m roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=imu_only  &
(sleep 30s && echo "Run trajectory query" && timeout 30s rosservice call /trajectory_query 0 > tq_$1_imu-$2_lidar-$3_imu_only.txt)
sleep 5s
python ~/kobuki_sim/bagfiles/tq_parse.py tq_$1_imu-$2_lidar-$3_imu_only.txt
python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 imu_only
wait


#run lidar error
timeout 2m roslaunch kobuki_yd cartographer_bag_record.launch bag_filenames:=$1 imu_noise:=$2 lidar_noise:=$3 config:=lidar_only  &
(sleep 30s && echo "Run trajectory query" && timeout 30s rosservice call /trajectory_query 0 > tq_$1_imu-$2_lidar-$3_lidar_only.txt)
sleep 5s
python ~/kobuki_sim/bagfiles/tq_parse.py tq_$1_imu-$2_lidar-$3_lidar_only.txt
python ~/kobuki_sim/bagfiles/line_trajectory.py $1 $2 $3 lidar_only
wait
