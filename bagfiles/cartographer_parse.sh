rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/bagfiles/record_files/output.bag.pbstream'}"
roslaunch cartographer_ros assets_writer_backpack_2d.launch bag_filenames:= ${HOME}/bagfiles/record_files/output.bag pose_graph_filename:=${HOME}/bagfiles/record_files/output.bag.pbstream


python ~/kobuki_sim/bagfiles/bagFileRead.py -i output.bag
python ~/kobuki_sim/bagfiles/tf_parse_trajectory.py tf_link.txt
python ~/kobuki_sim/bagfiles/tf_parse_trajectory.py tf_footprint.txt
