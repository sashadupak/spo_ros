#!/bin/bash
cmd_ros="ros2 bag play ~/bags/rosbag2_2021_04_02-11_06_42/rosbag2_2021_04_02-11_06_42_0.db3"
while [ 1 ]
do
  eval $cmd_ros
  sleep 1
done
#endless play of a bag