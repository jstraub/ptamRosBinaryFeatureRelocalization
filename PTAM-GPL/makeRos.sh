#!/bin/bash

# compile the ros wrapper for th connection of PTAM to ros

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/ptam_ros_wrapper 
rosmake ptam_ros_wrapper
cp ptam_ros_wrapper/lib/libptam_ros_wrapper.so ./lib

