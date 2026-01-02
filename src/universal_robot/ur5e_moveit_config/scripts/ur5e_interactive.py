#!/usr/bin/env python3
"""
UR5e Interactive Control with Gazebo Visualization
Tính năng:
- Nhấn số 1-5 để thực hiện các chức năng khác nhau
- Hiển thị robot trong Gazebo
- Tránh vật cản tự động
"""

import sys
import rospy
import moveit_commander
import random
import math
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class UR5eInteractiveControl:
    def __init__(self):
        rospy.init_node("ur5e_interactive_control", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        
        rospy.sleep(2)

        # Cấu hình planning
        self.arm.set_planning_time(8.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)

        self.reference_frame = self.arm.get_planning_frame()
        
        # Publisher cho markers
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )

        rospy.sleep(1)

        # Định nghĩa các vị trí quan trọng
        self.positions = {
            "home": [0.3, 0.0, 0.4],
            "pick": [0.4, 0.2, 0.3],
            "place": [0.4, -0.2, 0.3],
            "inspect": [0.35, 0.0, 0.5],
            "safe": [0.25, 0.0, 0.45]
        }

        self.obstacle_created = False
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("UR5e INTERACTIVE CONTROL - SẴN SÀNG")
        rospy.loginfo("=" * 60)

    def clear_all_objects(self):
        """Xóa tất cả objects trong scene"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        # Xóa tất cả markers
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        
        rospy.sleep(0.5)
        rospy.loginfo("✓ Đã xóa tất cả objects")

    def draw_sphere_marker(self, position, color, label, marker_id):
        """Vẽ sphere marker tại vị trí"""
        # Sphere marker
        m = Marker()
        m.header.frame_id = self.reference_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "points"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        
        m.pose.position.x = position[0]
        m.pose.position.y = position[1]
        m.pose.position.z = position[2]
        m.pose.orientation.w = 1.0
        
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08
        
        m.color.r, m.color.g, m.color.b, m.color.a = color
        
        self.marker_pub.publish(m)
        
        # Text marker
        t = Marker()
        t.header.frame_id = self.reference_frame
        t.header.stamp = rospy.Time.now()
        t.ns = "labels"
        t.id = marker_id + 100
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        
        t.pose.position.x = position[0]
        t.pose.position.y = position[1]
        t.pose.position.z = position[2] + 0.12
        
        t.scale.z = 0.06
        t.color.r = t.color.g = t.color.b = 1.0
        t.color.a = 1.0
        t.text = label
        
        self.marker_pub.publish(t)

    def move_to_position(self, pos, label="TARGET"):
        """Di chuyển đến vị trí"""
        rospy.loginfo(f"→ Di chuyển đến {label}...")
        
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        
        # Orientation (hướng xuống)
        q = quaternion_from_euler(-math.pi/2, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        
        if success:
            rospy.loginfo(f"✓ Đã đến {label}")
        else:
            rospy.logwarn(f"✗ Không thể đến {label}")
        
        return success

    def create_obstacles(self, num=2):
        """Tạo vật cản ngẫu nhiên"""
        rospy.loginfo(f"Tạo {num} vật cản...")
        
        self.clear_all_objects()
        
        for i in range(num):
            x = random.uniform(0.28, 0.42)
            y = random.uniform(-0.15, 0.15)
            z = random.uniform(0.25, 0.38)
            size = random.uniform(0.06, 0.10)
            
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            
            name = f"obstacle_{i}"
            self.scene.add_box(name, pose, (size, size, size))
            
            rospy.loginfo(f"  + Vật cản {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        rospy.sleep(1)
        self.obstacle_created = True
        rospy.loginfo("✓ Đã tạo vật cản")

    # ============================================
    # CÁC CHỨC NĂNG CHÍNH (1-5)
    # ============================================
    
    def function_1_home(self):
        """1. Về vị trí HOME"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("CHỨC NĂNG 1: VỀ VỊ TRÍ HOME")
        rospy.loginfo("="*60)
        
        self.clear_all_objects()
        self.draw_sphere_marker(self.positions["home"], (0, 1, 0, 1), "HOME", 1)
        self.move_to_position(self.positions["home"], "HOME")

    def function_2_pick_place(self):
        """2. Pick & Place với vật cản"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("CHỨC NĂNG 2: PICK & PLACE")
        rospy.loginfo("="*60)
        
        self.clear_all_objects()
        
        # Vẽ markers
        self.draw_sphere_marker(self.positions["pick"], (0, 0, 1, 1), "PICK", 1)
        self.draw_sphere_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 2)
        
        # Tạo vật cản
        self.create_obstacles(2)
        
        # Thực hiện pick & place
        self.move_to_position(self.positions["pick"], "PICK")
        rospy.sleep(1)
        
        self.move_to_position(self.positions["place"], "PLACE")
        rospy.sleep(1)
        
        self.move_to_position(self.positions["home"], "HOME")

    def function_3_inspection(self):
        """3. Kiểm tra (Inspection) nhiều điểm"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("CHỨC NĂNG 3: KIỂM TRA NHIỀU ĐIỂM")
        rospy.loginfo("="*60)
        
        self.clear_all_objects()
        
        # Các điểm kiểm tra
        inspect_points = [
            ([0.35, 0.15, 0.45], "ĐIỂM 1"),
            ([0.35, 0.0, 0.5], "ĐIỂM 2"),
            ([0.35, -0.15, 0.45], "ĐIỂM 3"),
        ]
        
        # Vẽ markers
        for i, (pos, label) in enumerate(inspect_points):
            color = (1, 1, 0, 1)  # Màu vàng
            self.draw_sphere_marker(pos, color, label, i+1)
        
        # Di chuyển qua các điểm
        for pos, label in inspect_points:
            self.move_to_position(pos, label)
            rospy.sleep(1)
        
        self.move_to_position(self.positions["home"], "HOME")

    def function_4_random_motion(self):
        """4. Di chuyển ngẫu nhiên tránh vật cản"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("CHỨC NĂNG 4: DI CHUYỂN NGẪU NHIÊN")
        rospy.loginfo("="*60)
        
        self.create_obstacles(3)
        
        # Tạo 4 điểm ngẫu nhiên
        for i in range(4):
            x = random.uniform(0.28, 0.40)
            y = random.uniform(-0.2, 0.2)
            z = random.uniform(0.30, 0.48)
            
            pos = [x, y, z]
            self.draw_sphere_marker(pos, (1, 0, 1, 1), f"P{i+1}", i+10)
            self.move_to_position(pos, f"Điểm {i+1}")
            rospy.sleep(0.5)
        
        self.move_to_position(self.positions["home"], "HOME")

    def function_5_demo_full(self):
        """5. Demo đầy đủ tất cả chức năng"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("CHỨC NĂNG 5: DEMO ĐẦY ĐỦ")
        rospy.loginfo("="*60)
        
        rospy.loginfo("\n[1/4] Home...")
        self.function_1_home()
        rospy.sleep(2)
        
        rospy.loginfo("\n[2/4] Pick & Place...")
        self.function_2_pick_place()
        rospy.sleep(2)
        
        rospy.loginfo("\n[3/4] Inspection...")
        self.function_3_inspection()
        rospy.sleep(2)
        
        rospy.loginfo("\n[4/4] Random Motion...")
        self.function_4_random_motion()
        
        rospy.loginfo("\n✓ HOÀN THÀNH DEMO ĐẦY ĐỦ!")

    def show_menu(self):
        """Hiển thị menu"""
        print("\n" + "="*60)
        print("UR5e INTERACTIVE CONTROL - MENU ĐIỀU KHIỂN")
        print("="*60)
        print("1. Về vị trí HOME")
        print("2. Pick & Place (có vật cản)")
        print("3. Kiểm tra nhiều điểm (Inspection)")
        print("4. Di chuyển ngẫu nhiên (tránh vật cản)")
        print("5. Demo đầy đủ tất cả chức năng")
        print("-"*60)
        print("0. Xóa tất cả objects")
        print("q. Thoát")
        print("="*60)

    def run(self):
        """Chạy chương trình chính"""
        self.show_menu()
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n>>> Nhập lệnh (1-5, 0, q): ").strip().lower()
                
                if cmd == '1':
                    self.function_1_home()
                    
                elif cmd == '2':
                    self.function_2_pick_place()
                    
                elif cmd == '3':
                    self.function_3_inspection()
                    
                elif cmd == '4':
                    self.function_4_random_motion()
                    
                elif cmd == '5':
                    self.function_5_demo_full()
                    
                elif cmd == '0':
                    self.clear_all_objects()
                    
                elif cmd in ['q', 'quit', 'exit']:
                    rospy.loginfo("Đang thoát...")
                    break
                    
                else:
                    print("⚠ Lệnh không hợp lệ! Vui lòng nhập 1-5, 0 hoặc q")
                    
            except KeyboardInterrupt:
                rospy.loginfo("\n\nĐã dừng bởi người dùng")
                break
            except Exception as e:
                rospy.logerr(f"Lỗi: {e}")


def main():
    try:
        controller = UR5eInteractiveControl()
        controller.run()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\nĐã thoát")


if __name__ == "__main__":
    main()
