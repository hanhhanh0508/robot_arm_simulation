#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script ULTRA SAFE - Timeout cực lớn, chậm nhất có thể
"""
import sys
import rospy
import moveit_commander
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

class UltraSafePickPlace:
    def __init__(self):
        rospy.init_node("ultra_safe_pick_place", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        
        # ✅ CẤU HÌNH CỰC CHẬM VÀ AN TOÀN
        self.arm.set_planning_time(30.0)        # 30 giây
        self.arm.set_num_planning_attempts(50)  # 50 lần thử
        self.arm.set_max_velocity_scaling_factor(0.05)   # Chỉ 5% tốc độ!
        self.arm.set_max_acceleration_scaling_factor(0.05)
        self.arm.set_goal_position_tolerance(0.1)  # Rất nới
        self.arm.set_goal_orientation_tolerance(0.3)
        
        # Kết nối controller với TIMEOUT CỰC LỚN
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
        
        self.current_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.sleep(2.0)
        
        # Vị trí AN TOÀN
        self.positions = {
            "home":  [0.3, 0.0, 0.5],     # Cao hơn
            "pick":  [0.45, 0.25, 0.45],  # Cao hơn
            "place": [0.45, -0.25, 0.45],
        }
        
        rospy.loginfo("✓ HỆ THỐNG SẴN SÀNG!")
    
    def joint_state_callback(self, msg):
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
    
    def execute_trajectory_ultra_safe(self, trajectory):
        """Execute với TIMEOUT CỰC LỚN"""
        if not trajectory.joint_trajectory.points:
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        
        # ✅ THÊM THỜI GIAN CHO MỖI ĐIỂM
        for point in goal.trajectory.points:
            original_time = point.time_from_start.to_sec()
            point.time_from_start = rospy.Duration(original_time * 5.0)  # x5!
        
        rospy.loginfo("  📤 Execute (ULTRA SAFE mode)...")
        self.client.send_goal(goal)
        
        # ✅ TIMEOUT = 60 GIÂY!
        timeout = 60.0
        rospy.loginfo("  ⏱️  Đợi tối đa %.1f giây..." % timeout)
        
        success = self.client.wait_for_result(timeout=rospy.Duration(timeout))
        
        if success:
            rospy.loginfo("  ✅ Hoàn thành!")
            return True
        else:
            rospy.logerr("  ❌ Timeout!")
            return False
    
    def move_to_position(self, pos, label="TARGET"):
        """Di chuyển với RETRY vô hạn"""
        rospy.loginfo("\n→ Di chuyển: %s (%.2f, %.2f, %.2f)" % 
                     (label, pos[0], pos[1], pos[2]))
        
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        
        q = quaternion_from_euler(-math.pi/2, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        attempt = 0
        while not rospy.is_shutdown():
            attempt += 1
            self.arm.set_pose_target(pose)
            
            rospy.loginfo("  Lần thử %d..." % attempt)
            plan = self.arm.plan()
            
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = bool(plan.joint_trajectory.points)
                trajectory = plan
            
            self.arm.clear_pose_targets()
            
            if success:
                rospy.loginfo("  ✓ Plan OK!")
                if self.execute_trajectory_ultra_safe(trajectory):
                    return True
                else:
                    rospy.logwarn("  ⚠️ Execute thất bại, retry...")
            else:
                rospy.logwarn("  ⚠️ Planning thất bại, retry...")
            
            rospy.sleep(2.0)
        
        return False
    
    def demo_simple(self):
        """Demo đơn giản nhất"""
        rospy.loginfo("\n🚀 DEMO SIMPLE")
        
        # Chỉ di chuyển HOME → PICK → HOME
        if not self.move_to_position(self.positions["home"], "HOME"):
            return
        rospy.sleep(3)
        
        if not self.move_to_position(self.positions["pick"], "PICK"):
            return
        rospy.sleep(3)
        
        if not self.move_to_position(self.positions["home"], "HOME"):
            return
        
        rospy.loginfo("✅ HOÀN THÀNH!")

def main():
    try:
        controller = UltraSafePickPlace()
        
        print("\n" + "="*70)
        print("🐢 ULTRA SAFE MODE - CHẬM NHẤT CÓ THỂ")
        print("="*70)
        print("✅ Tốc độ: 5% (rất chậm)")
        print("✅ Timeout: 60 giây/bước")
        print("✅ Retry: Vô hạn")
        print("="*70)
        
        input("\nẤn Enter để bắt đầu...")
        controller.demo_simple()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
