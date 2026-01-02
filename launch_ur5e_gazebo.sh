#!/bin/bash

# Script khởi động UR5e trong Gazebo với MoveIt

# 1. Launch Gazebo với UR5e
echo "Starting Gazebo with UR5e robot..."
roslaunch ur_gazebo ur5e_bringup.launch &
sleep 10

# 2. Launch MoveIt cho planning
echo "Starting MoveIt..."
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true &
sleep 5

# 3. Launch RViz để visualization
echo "Starting RViz..."
roslaunch ur5e_moveit_config moveit_rviz.launch &

echo "System ready! You can now run the demo script."
