#roslaunch cartographer_ros assets_writer_backpack_2d.launch bag_filenames := ${HOME}/bagfiles/record_files/$1.bag pose_graph_filename := ${HOME}/bagfiles/record_files/$1.bag.pbstream


#python ~/kobuki_sim/bagfiles/bagFileRead.py -i output.bag
#python ~/kobuki_sim/bagfiles/tf_parse_trajectory.py tf_link.txt
python ~/kobuki_sim/bagfiles/tf_parse_trajectory.py tf_footprint.txt
