UR5e MoveIt Demo

Demo điều khiển robot UR5e bằng MoveIt (ROS Noetic) với:

Hiển thị điểm HOME và TARGET

Tạo 2 vật cản ngẫu nhiên

Robot tự lập kế hoạch và né vật cản

Chạy ở chế độ fake execution (RViz)

Yêu cầu

Ubuntu + ROS Noetic

MoveIt

universal\_robot

ur5e\_moveit\_config

Chạy demo

Mở terminal 1

source ~/ur5\_ws/devel/setup.bash

roslaunch ur5e\_moveit\_config demo.launch



Mở terminal 2

source ~/ur5\_ws/devel/setup.bash

rosrun ur5e\_moveit\_config ur5\_moveit\_control.py

Chức năng chính

Hiển thị HOME / TARGET bằng marker

Sinh 2 vật cản ngẫu nhiên

Robot tự tìm đường tránh vật cản

Điều chỉnh tốc độ \& thời gian planning

Chạy thử di chuyển đơn giản

Ghi chú

Robot chỉ mô phỏng (fake controller)

Chuyển động hiển thị trong RViz

###### ***~~~RViz CÓ VẬT CẢN VÀ TRÁNH VA CHẠM~~~***

<i>cd project</i>

<i>catkin\_make clean</i>

<i>catkin\_make</i>

<i>source devel/setup.bash</i>

<i># Terminal 1: Khởi động Gazebo + UR5e</i>

<i>roslaunch ur\_gazebo ur5e\_bringup.launch</i>



<i># Terminal 2: MoveIt planning</i>

<i>roslaunch ur5e\_moveit\_config moveit\_planning\_execution.launch sim:=true</i>



<i># Terminal 3: RViz visualization</i>

<i>roslaunch ur5e\_moveit\_config moveit\_rviz.launch config:=$(rospack find ur5e\_moveit\_config)/launch/moveit.rviz</i>



<i># Terminal 4: Chạy script pick \& place</i>

<i>rosrun ur5e\_moveit\_config ur5e\_with\_obstacles\_working.py</i>

###### <i>**~~~**</i>

