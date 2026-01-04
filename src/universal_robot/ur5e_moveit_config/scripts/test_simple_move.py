#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script test đơn giản - chỉ di chuyển lên/xuống
"""
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander

class SimpleTest:
    def __init__(self):
        rospy.init_node("simple_move_test", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Khởi tạo move group
        self.arm = MoveGroupCommander("manipulator")
        
        # CẤU HÌNH ĐƠN GIẢN
        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(3)
        self.arm.set_max_velocity_scaling_factor(0.1)  # Chậm lại
        self.arm.set_max_acceleration_scaling_factor(0.1)
        
        # Tăng tolerance để dễ đạt mục tiêu
        self.arm.set_goal_position_tolerance(0.05)  # 5cm
        self.arm.set_goal_orientation_tolerance(0.2)  # ~11 độ
        
        rospy.loginfo("✓ Khởi tạo xong!")
        rospy.loginfo("✓ Controller: %s" % self.arm.get_name())
        
    def get_current_joints(self):
        """Lấy góc khớp hiện tại"""
        joints = self.arm.get_current_joint_values()
        rospy.loginfo("Góc khớp hiện tại:")
        for i, val in enumerate(joints):
            rospy.loginfo("  Khớp %d: %.3f rad (%.1f độ)" % (i+1, val, val*57.3))
        return joints
    
    def move_joint_angles(self, joint_values, description=""):
        """Di chuyển đến góc khớp cụ thể"""
        rospy.loginfo("\n→ Di chuyển: %s" % description)
        
        self.arm.set_joint_value_target(joint_values)
        
        # Plan
        rospy.loginfo("  Planning...")
        plan = self.arm.plan()
        
        # Kiểm tra plan thành công không
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = bool(plan.joint_trajectory.points)
            trajectory = plan
            
        if not success:
            rospy.logerr("✗ Planning thất bại!")
            return False
        
        rospy.loginfo("  ✓ Plan OK! Executing...")
        
        # Execute
        result = self.arm.execute(trajectory, wait=True)
        self.arm.stop()
        
        if result:
            rospy.loginfo("  ✓ Thành công!")
            return True
        else:
            rospy.logerr("  ✗ Execution thất bại!")
            return False
    
    def test_simple_moves(self):
        """Test các di chuyển đơn giản"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("BẮT ĐẦU TEST DI CHUYỂN ĐơN GIẢN")
        rospy.loginfo("="*60)
        
        # Lấy vị trí ban đầu
        rospy.loginfo("\n1️⃣ VỊ TRÍ HIỆN TẠI:")
        start_joints = self.get_current_joints()
        
        input("\nẤn Enter để di chuyển lên (nâng khớp 2)...")
        
        # Test 1: Nâng khớp shoulder_lift lên
        rospy.loginfo("\n2️⃣ TEST: Nâng khớp 2 lên 30 độ")
        joints_up = list(start_joints)
        joints_up[1] = -0.5  # shoulder_lift: -0.5 rad = -28.6 độ (nâng lên)
        
        success = self.move_joint_angles(joints_up, "Nâng lên")
        
        if not success:
            rospy.logerr("Test thất bại!")
            return
        
        rospy.sleep(2)
        
        input("\nẤn Enter để di chuyển xuống...")
        
        # Test 2: Hạ khớp shoulder_lift xuống
        rospy.loginfo("\n3️⃣ TEST: Hạ khớp 2 xuống")
        joints_down = list(start_joints)
        joints_down[1] = 0.5  # shoulder_lift: 0.5 rad = 28.6 độ (hạ xuống)
        
        success = self.move_joint_angles(joints_down, "Hạ xuống")
        
        if not success:
            rospy.logerr("Test thất bại!")
            return
        
        rospy.sleep(2)
        
        input("\nẤn Enter để về vị trí ban đầu...")
        
        # Test 3: Về vị trí ban đầu
        rospy.loginfo("\n4️⃣ TEST: Về vị trí ban đầu")
        success = self.move_joint_angles(start_joints, "Về vị trí ban đầu")
        
        if success:
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo("✓ ✓ ✓ TẤT CẢ TEST THÀNH CÔNG! ✓ ✓ ✓")
            rospy.loginfo("="*60)
        else:
            rospy.logerr("\n✗ Test thất bại!")

def main():
    try:
        tester = SimpleTest()
        
        print("\n" + "="*60)
        print("CHƯƠNG TRÌNH TEST ĐƠN GIẢN")
        print("="*60)
        print("Chương trình sẽ:")
        print("1. Hiển thị vị trí hiện tại")
        print("2. Di chuyển lên (nâng khớp 2)")
        print("3. Di chuyển xuống (hạ khớp 2)")
        print("4. Về vị trí ban đầu")
        print("="*60)
        
        input("\nẤn Enter để bắt đầu...")
        
        tester.test_simple_moves()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nDừng chương trình")

if __name__ == "__main__":
    main()
