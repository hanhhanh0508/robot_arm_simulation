#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script ĐƠN GIẢN NHẤT - Chỉ di chuyển lên/xuống (ĐÃ FIX!)
"""
import sys
import rospy
import actionlib
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

def normalize_angle(angle):
    """Chuẩn hóa góc về [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class SimpleUpDown:
    def __init__(self):
        rospy.init_node("simple_updown", anonymous=True)
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Kết nối controller
        rospy.loginfo("Đang kết nối controller...")
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("✗ Không tìm thấy controller!")
            return
        
        rospy.loginfo("✓ Kết nối controller thành công!")
        
        # Nhận joint states
        self.current_joints = None
        self.raw_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        rospy.sleep(1.5)
        
        if self.current_joints is None:
            rospy.logerr("✗ Không nhận được joint states!")
            return
        
        # Kiểm tra và fix
        self.check_and_fix_angles()
        
        rospy.loginfo("="*70)
        rospy.loginfo("✓ SẴN SÀNG!")
        rospy.loginfo("="*70)
    
    def joint_state_callback(self, msg):
        """Callback - normalize góc tự động"""
        self.raw_joints = {}
        self.current_joints = {}
        
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                raw = msg.position[i]
                normalized = normalize_angle(raw)
                
                self.raw_joints[name] = raw
                self.current_joints[name] = normalized
    
    def check_and_fix_angles(self):
        """Kiểm tra và fix góc không hợp lệ"""
        rospy.loginfo("\n🔍 Kiểm tra góc khớp...")
        
        invalid = False
        for name in self.joint_names:
            raw = self.raw_joints[name]
            normalized = self.current_joints[name]
            
            if abs(raw) > 10.0:
                rospy.logwarn("  ⚠️  %s: %.1f rad → Fix: %.2f rad" % (name, raw, normalized))
                invalid = True
            else:
                rospy.loginfo("  ✓ %s: %.2f rad (%.0f°)" % (name, raw, raw*57.3))
        
        if invalid:
            rospy.logwarn("\n⚠️  Đang fix góc khớp...")
            normalized_values = [self.current_joints[name] for name in self.joint_names]
            
            if self.send_trajectory(normalized_values, duration=2.0):
                rospy.loginfo("✅ Fix thành công!\n")
                rospy.sleep(1.0)
            else:
                rospy.logerr("❌ Fix thất bại!\n")
        else:
            rospy.loginfo("✅ Tất cả góc khớp hợp lệ\n")
    
    def send_trajectory(self, target_joints, duration=3.0):
        """Gửi trajectory đến controller"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # Điểm 1: Hiện tại
        point1 = JointTrajectoryPoint()
        point1.positions = [self.current_joints[name] for name in self.joint_names]
        point1.velocities = [0.0] * 6
        point1.time_from_start = rospy.Duration(0.0)
        
        # Điểm 2: Đích
        point2 = JointTrajectoryPoint()
        point2.positions = target_joints
        point2.velocities = [0.0] * 6
        point2.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points = [point1, point2]
        
        # Gửi
        self.client.send_goal(goal)
        success = self.client.wait_for_result(timeout=rospy.Duration(duration + 2.0))
        
        if success:
            result = self.client.get_result()
            return True
        return False
    
    def test_updown(self):
        """Test di chuyển lên/xuống"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🚀 BẮT ĐẦU TEST DI CHUYỂN LÊN/XUỐNG")
        rospy.loginfo("="*70)
        
        # Lưu vị trí ban đầu
        start_joints = [self.current_joints[name] for name in self.joint_names]
        
        rospy.loginfo("\n📍 Vị trí ban đầu:")
        for i, val in enumerate(start_joints):
            rospy.loginfo("  Khớp %d: %.2f rad (%.0f°)" % (i+1, val, val*57.3))
        
        input("\n⏸️  Ấn Enter để NÂNG LÊN...")
        
        # Test 1: Nâng lên
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("1️⃣  NÂNG LÊN (khớp 2: -45°)")
        rospy.loginfo("="*70)
        
        joints_up = list(start_joints)
        joints_up[1] = -0.785  # -45 độ
        
        rospy.loginfo("Gửi lệnh...")
        if self.send_trajectory(joints_up, duration=3.0):
            rospy.loginfo("✅ Thành công!")
        else:
            rospy.logerr("❌ Thất bại!")
            return
        
        rospy.sleep(2.0)
        
        input("\n⏸️  Ấn Enter để HẠ XUỐNG...")
        
        # Test 2: Hạ xuống
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("2️⃣  HẠ XUỐNG (khớp 2: +45°)")
        rospy.loginfo("="*70)
        
        joints_down = list(start_joints)
        joints_down[1] = 0.785  # +45 độ
        
        rospy.loginfo("Gửi lệnh...")
        if self.send_trajectory(joints_down, duration=3.0):
            rospy.loginfo("✅ Thành công!")
        else:
            rospy.logerr("❌ Thất bại!")
            return
        
        rospy.sleep(2.0)
        
        input("\n⏸️  Ấn Enter để VỀ VỊ TRÍ BAN ĐẦU...")
        
        # Test 3: Về ban đầu
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("3️⃣  VỀ VỊ TRÍ BAN ĐẦU")
        rospy.loginfo("="*70)
        
        rospy.loginfo("Gửi lệnh...")
        if self.send_trajectory(start_joints, duration=3.0):
            rospy.loginfo("✅ Thành công!")
        else:
            rospy.logerr("❌ Thất bại!")
            return
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🎉 🎉 🎉 TẤT CẢ TEST THÀNH CÔNG! 🎉 🎉 🎉")
        rospy.loginfo("="*70)

def main():
    try:
        tester = SimpleUpDown()
        
        print("\n" + "="*70)
        print("🤖 TEST ĐƠN GIẢN: DI CHUYỂN LÊN/XUỐNG (ĐÃ FIX!)")
        print("="*70)
        print("✅ Tự động fix góc khớp không hợp lệ")
        print("✅ KHÔNG cần MoveIt planning")
        print("✅ Gửi lệnh trực tiếp đến controller")
        print("="*70)
        print("\nChương trình sẽ:")
        print("  1. Kiểm tra và fix vị trí ban đầu")
        print("  2. Nâng robot lên (khớp 2: -45°)")
        print("  3. Hạ robot xuống (khớp 2: +45°)")
        print("  4. Về vị trí ban đầu")
        print("="*70)
        
        input("\n⏸️  Ấn Enter để bắt đầu...")
        
        tester.test_updown()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\n👋 Thoát chương trình")

if __name__ == "__main__":
    main()
