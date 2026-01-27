#!/bin/bash
#
# This file builds the ROS2 X86 gita test node for testing the ROS2 gita
#
# Copyright (c) 2025 Piaggio Fast Forward (PFF), Inc.
# All Rights Reserved. Reproduction, publication,
# or re-transmittal not allowed without written permission of PFF, Inc.
# source the ros setup script
source "/opt/ros/foxy/setup.bash"

# if the packages don't already exist, create them using the following; note you will have to change the C++ version to CPP17 in the CMakeLists.txt files generated
#ros2 pkg create --build-type ament_cmake --node-name gita_test_node gita_test --dependencies rclcpp geometry_msgs
#ros2 pkg create --build-type ament_cmake --node-name gita_node gita --dependencies rclcpp geometry_msg

# if for some reason this suddenly doesn't work and you get 'catkin_pkg not found' errors, run 'pip install catkin_pkg'

# As you add additional ROS2 depenencies, you will need to add lines for the dependencies after the <buildtool_depend>ament_cmake</buildtool_depend> line in node/package.xml; e.g.:
#   <buildtool_depend>ament_cmake</buildtool_depend>
#   <depend>rclcpp</depend>
#   <depend>geometry_msgs</depend>
#
# You will also need to additional lines to node/CMakeLists.txt for each package, e.g.:
#   find_package(ament_cmake REQUIRED) # this is in by default
#   find_package(rclcpp REQUIRED)
#   find_package(geometry_msgs REQUIRED)
# 
# And also the target_include_directories:
#
# target_include_directories(gita_test_node PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)
#    ament_target_dependencies(
#    node_name
#    "rclcpp"
#    "geometry_msgs"

# record the current directory
OG_DIR=$(echo "$PWD")
# change to the developer/ros2 directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR
echo $SCRIPT_DIR

# build the gita test package
colcon build --packages-select gita_test

cd $OG_DIR

# to run the newly built package:
# X86 (ran form workspace) - source "/opt/ros/foxy/setup.bash"; source install/setup.sh
#  ros2 run <package_name> <node_name> # note that node name has to be the same as the string that was passed to the Node parent class for your custom node

