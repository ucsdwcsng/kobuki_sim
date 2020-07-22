mkdir ~/bagfiles/record_files_$1
cd ~/bagfiles/record_files_$1

roslaunch kobuki_yd record_bag_files.launch bagfile_name:=$1


