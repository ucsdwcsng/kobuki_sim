# kobuki_sim

This repository is for running cartographer mapping simulations of the kobuki robot. You will be able to run a simulation of a robot moving across an enviornment, then map the enviornment using the IMU and Lidar data, then test the accuracy of the map by comparing the localized trajectory of the robot with the actual trajectory.


Installation Instructions:

1. Install ros-kinetic: instructions here -> http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Install cartographer-turtlebot: instructions here -> https://google-cartographer-ros-for-turtlebots.readthedocs.io/en/latest/
3. Install gazebo7: instructions here -> http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1
4. Install the kobuki packages -> http://wiki.ros.org/kobuki/Tutorials/Installation
5. Install the ydlidar package -> https://github.com/YDLIDAR/ydlidar_ros
6. Install robot_pose_ekf with the command: sudo apt-get install ros-kinetic-robot_pose_ekf
7. Replace the folders for ydlidar, kobuki_description, and kobuki_gazebo in /opt/ros/kinetic/share with the corresponding folders in this repository
8. Place the kobuki_yd folder in /opt/ros/kinetic/share
9. Create a package for the trajectory commands circle.py and line.py in the bagfiles folder: instructions here -> http://wiki.ros.org/ROS/Tutorials/CreatingPackage
10. Create a bagfiles folder in the main directory

After this, you should be able to run a simulation for the kobuki in some enviornment. You are able to change the enviornment the robot is in by changing the world specified in the launch files in the kobuki_yd package. 

To run simulations, create a folder in the bagfiles folder in the main directory, then create an input.csv file. In this csv file, write your rows of inputs. The format of each row should be (imu standard deviation noise,lidar standard deviation noise,0,0,0) [no spaces]. Then create a copy of the input.csv file and name it output.csv. This file will be where the error values are outputted. The format of output.csv is (imu standard deviation,lidar standard deviation, combined error, imu only error, lidar scan matching only error). When you have input.csv and output.csv created, run trajectory_record.py to iterate through all of your inputted noise values.
