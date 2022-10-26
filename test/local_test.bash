#!/bin/bash -xv

### DOWNLOAD ROSBAG ###
cd ~/catkin_ws/src/ground_filter/bags/
wget "https://drive.google.com/uc?export=download&id=1wcGRP6KUx1Y-dQCjdHEJSdJeAIsMGTQ0" -O rosbag.bag

### ROS TEST ###
cd ~/catkin_ws/ && catkin_make run_tests_ground_filter_rostest_test_ground_filter_launch.test