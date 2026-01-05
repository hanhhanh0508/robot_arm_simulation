#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script ĐƠN GIẢN NHẤT - Di chuyển đến joint angles thay vì Cartesian
Tránh planning phức tạp
"""
import sys
import rospy
import moveit_commander
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class SuperSimpleControl:
    def __init__(self):
        rospy.init_node("super_simple_control", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # MoveIt
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        
        rospy.sleep(1)
        
        # ✅ CẤU HÌNH TỐI ƯU
        self.arm.set_planning_time(30.0)        # 30 giây planning
        self.arm.set_num_planning_attempts(50)
        self.arm.set_max_velocity_scaling_factor(0.05)   # 5% tốc độ
        self.arm.set_max_acceleration_scaling_factor(0.05)
        self.arm.set_goal_joint_tolerance(0.01)  # 0.01 rad tolerance
        
        # Action client
        rospy.loginfo("Kết nối controller...")
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self.client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("✗ Không tìm thấy controller!")
            return
        
        rospy.loginfo("✓ Kết nối controller OK!")
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Joint states
        self.current_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.sleep(1.0)
        
        # ✅ Định nghĩa joint positions an toàn
        self.positions = {
            "home":  [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],  # Home position
            "up":    [0.0, -0.5, 0.5, -1.57, -1.57, 0.0],    # Nâng lên
            "down":  [0.0, -2.0, 2.0, -1.57, -1.57, 0.0],    # Hạ xuống
        }
        
        rospy.loginfo("="*70)
        rospy.loginfo("✓ HỆ THỐNG SẴN SÀNG!")
        rospy.loginfo("="*70)
    
    def joint_state_callback(self, msg):
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
    
    def execute_trajectory_direct(self, trajectory, label=""):
        """Execute trajectory với timeout lớn"""
        if not trajectory.joint_trajectory.points:
            rospy.logerr("  ❌ Trajectory rỗng!")
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        
        # ✅ THÊM THỜI GIAN cho mỗi điểm (x5)
        for point in goal.trajectory.points:
            original_time = point.time_from_start.to_sec()
            point.time_from_start = rospy.Duration(original_time * 5.0)
        
        # Tính tổng thời gian
        if goal.trajectory.points:
            duration = goal.trajectory.points[-1].time_from_start.to_sec()
        else:
            duration = 10.0
        
        rospy.loginfo("  📤 Execute %s (%.1f giây)..." % (label, duration))
        self.client.send_goal(goal)
        
        # ✅ TIMEOUT = duration * 2
        timeout = duration * 2.0
        success = self.client.wait_for_result(timeout=rospy.Duration(timeout))
        
        if success:
            result = self.client.get_result()
            rospy.loginfo("  ✅ Hoàn thành!")
            return True
        else:
            rospy.logerr("  ❌ Timeout sau %.1f giây!" % timeout)
            return False
    
    def move_to_joint_angles(self, joint_values, label="VỊ TRÍ"):
        """Di chuyển đến góc khớp - ĐƠN GIẢN NHẤT"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("→ Di chuyển đến: %s" % label)
        rospy.loginfo("="*70)
        
        # Hiển thị góc khớp
        rospy.loginfo("Góc khớp mục tiêu:")
        for i, val in enumerate(joint_values):
            rospy.loginfo("  Khớp %d: %.2f rad (%.0f°)" % (i+1, val, val*57.3))
        
        # ✅ Set joint target (đơn giản nhất)
        self.arm.set_joint_value_target(joint_values)
        
        rospy.loginfo("🔧 Planning...")
        plan = self.arm.plan()
        
        # Kiểm tra plan
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = bool(plan.joint_trajectory.points)
            trajectory = plan
        
        if not success:
            rospy.logerr("❌ Planning thất bại!")
            return False
        
        num_points = len(trajectory.joint_trajectory.points)
        if trajectory.joint_trajectory.points:
            duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            duration = 0
        
        rospy.loginfo("✅ Plan OK! (%d điểm, %.1f giây)" % (num_points, duration))
        
        # Execute
        return self.execute_trajectory_direct(trajectory, label)
    
    def demo_simple(self):
        """Demo đơn giản: HOME → UP → DOWN → HOME"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🚀 BẮT ĐẦU DEMO ĐƠN GIẢN")
        rospy.loginfo("="*70)
        
        # 1. Về HOME
        rospy.loginfo("\n1️⃣ VỀ HOME")
        if not self.move_to_joint_angles(self.positions["home"], "HOME"):
            rospy.logerr("❌ Không về được HOME")
            return
        rospy.sleep(3.0)
        
        # 2. Nâng lên
        rospy.loginfo("\n2️⃣ NÂNG LÊN")
        if not self.move_to_joint_angles(self.positions["up"], "UP"):
            rospy.logerr("❌ Không nâng được")
            return
        rospy.sleep(3.0)
        
        # 3. Hạ xuống
        rospy.loginfo("\n3️⃣ HẠ XUỐNG")
        if not self.move_to_joint_angles(self.positions["down"], "DOWN"):
            rospy.logerr("❌ Không hạ được")
            return
        rospy.sleep(3.0)
        
        # 4. Về HOME
        rospy.loginfo("\n4️⃣ VỀ HOME")
        if not self.move_to_joint_angles(self.positions["home"], "HOME"):
            rospy.logerr("❌ Không về được HOME")
            return
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🎉 🎉 🎉 HOÀN THÀNH DEMO! 🎉 🎉 🎉")
        rospy.loginfo("="*70)

def main():
    try:
        controller = SuperSimpleControl()
        
        print("\n" + "="*70)
        print("🤖 CHƯƠNG TRÌNH ĐƠN GIẢN NHẤT")
        print("="*70)
        print("✅ Sử dụng joint angles (không dùng Cartesian)")
        print("✅ Planning time: 30 giây")
        print("✅ Tốc độ: 5% (rất chậm)")
        print("="*70)
        print("\nChương trình sẽ:")
        print("  1. Về HOME")
        print("  2. Nâng lên")
        print("  3. Hạ xuống")
        print("  4. Về HOME")
        print("="*70)
        
        input("\n⏸️  Ấn Enter để bắt đầu...")
        
        controller.demo_simple()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\n👋 Thoát chương trình")

if __name__ == "__main__":
    main()
