#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class DirectTrajectoryTest:
    def __init__(self):
        rospy.init_node("direct_trajectory_test")
        
        # Tên các khớp
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Kết nối action client
        rospy.loginfo("Đang kết nối action server...")
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("✗ Không tìm thấy action server!")
            rospy.logerr("Kiểm tra controller có chạy không:")
            rospy.logerr("  rosservice call /controller_manager/list_controllers")
            return
        
        rospy.loginfo("✓ Kết nối action server thành công!")
        
        # Lấy vị trí hiện tại
        self.current_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        rospy.sleep(1.0)
        
        if self.current_joints is None:
            rospy.logerr("✗ Không nhận được joint states!")
            return
        
        rospy.loginfo("✓ Nhận được vị trí hiện tại")
        
    def joint_state_callback(self, msg):
        """Nhận joint states"""
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
    
    def send_trajectory(self, target_joints, duration=3.0):
        """Gửi trajectory đến controller"""
        if self.current_joints is None:
            rospy.logerr("✗ Chưa có vị trí hiện tại!")
            return False
        
        # Tạo goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # Điểm 1: Vị trí hiện tại (t=0)
        point1 = JointTrajectoryPoint()
        point1.positions = [self.current_joints[name] for name in self.joint_names]
        point1.velocities = [0.0] * 6
        point1.time_from_start = rospy.Duration(0.0)
        
        # Điểm 2: Vị trí đích (t=duration)
        point2 = JointTrajectoryPoint()
        point2.positions = target_joints
        point2.velocities = [0.0] * 6
        point2.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points = [point1, point2]
        
        # Gửi goal
        rospy.loginfo("Gửi trajectory...")
        self.client.send_goal(goal)
        
        # Đợi kết quả
        rospy.loginfo("Đang đợi kết quả...")
        success = self.client.wait_for_result(timeout=rospy.Duration(duration + 2.0))
        
        if success:
            result = self.client.get_result()
            rospy.loginfo("✓ Trajectory hoàn thành!")
            rospy.loginfo("  Error code: %d" % result.error_code)
            return True
        else:
            rospy.logerr("✗ Timeout hoặc thất bại!")
            return False
    
    def test_simple_motion(self):
        """Test di chuyển đơn giản"""
        if self.current_joints is None:
            rospy.logerr("Không thể test - không có joint states")
            return
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("BẮT ĐẦU TEST DI CHUYỂN")
        rospy.loginfo("="*60)
        
        # Vị trí hiện tại
        rospy.loginfo("\nVị trí hiện tại:")
        for name in self.joint_names:
            val = self.current_joints[name]
            rospy.loginfo("  %s: %.3f rad (%.1f độ)" % (name, val, val*57.3))
        
        input("\nẤn Enter để nâng khớp shoulder_lift lên...")
        
        # Test 1: Nâng khớp 2
        rospy.loginfo("\n1️⃣ Nâng khớp shoulder_lift lên -30 độ")
        target = [
            self.current_joints['shoulder_pan_joint'],
            -0.5,  # shoulder_lift: -0.5 rad = -28.6 độ
            self.current_joints['elbow_joint'],
            self.current_joints['wrist_1_joint'],
            self.current_joints['wrist_2_joint'],
            self.current_joints['wrist_3_joint']
        ]
        
        success = self.send_trajectory(target, duration=3.0)
        
        if not success:
            rospy.logerr("Test thất bại!")
            return
        
        rospy.sleep(1.0)
        
        input("\nẤn Enter để hạ xuống...")
        
        # Test 2: Hạ khớp 2
        rospy.loginfo("\n2️⃣ Hạ khớp shoulder_lift xuống +30 độ")
        target[1] = 0.5  # +0.5 rad = +28.6 độ
        
        success = self.send_trajectory(target, duration=3.0)
        
        if not success:
            rospy.logerr("Test thất bại!")
            return
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("✓ ✓ ✓ HOÀN THÀNH TEST! ✓ ✓ ✓")
        rospy.loginfo("="*60)

def main():
    try:
        tester = DirectTrajectoryTest()
        
        print("\n" + "="*60)
        print("TEST GỬI TRAJECTORY TRỰC TIẾP")
        print("="*60)
        print("Script này gửi trajectory trực tiếp đến controller")
        print("(Bỏ qua MoveIt để test controller có hoạt động không)")
        print("="*60)
        
        input("\nẤn Enter để bắt đầu...")
        
        tester.test_simple_motion()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nDừng chương trình")

if __name__ == "__main__":
    main()
