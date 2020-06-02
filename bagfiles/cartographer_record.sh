mkdir record_files
cd record_files


rosrun tf tf_echo /map /base_link 10 > tf_link.txt &
rosrun tf tf_echo /map /base_footprint 10 > tf_footprint.txt &
rosbag record -O output.bag /mobile_base/sensors/imu_data /gazebo/model_states /odom /robot_pose_ekf/odom_combined



