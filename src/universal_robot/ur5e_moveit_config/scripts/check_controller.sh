#!/bin/bash
# Script kiểm tra controller và topics

echo "=========================================="
echo "KIỂM TRA HỆ THỐNG CONTROLLER"
echo "=========================================="

echo -e "\n1️⃣ Danh sách controllers:"
rosservice call /controller_manager/list_controllers

echo -e "\n2️⃣ Kiểm tra topic joint_states:"
rostopic echo /joint_states -n 1

echo -e "\n3️⃣ Kiểm tra topic trajectory controller:"
rostopic info /scaled_pos_joint_traj_controller/follow_joint_trajectory

echo -e "\n4️⃣ Kiểm tra action server:"
rostopic list | grep follow_joint_trajectory

echo -e "\n5️⃣ MoveIt controller config:"
rosparam get /move_group/trajectory_execution

echo -e "\n=========================================="
echo "Hoàn thành kiểm tra!"
echo "=========================================="
