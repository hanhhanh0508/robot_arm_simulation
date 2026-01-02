#!/usr/bin/env python3
"""
UR5e Pick & Place Demo với tránh vật cản
Chạy robot trong Gazebo để nhặt và đặt đồ vật
"""

import sys
import rospy
import moveit_commander
import random
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface, MoveGroupCommander


class UR5ePickPlace:
    def __init__(self):
        rospy.init_node("ur5e_pick_place_demo", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Khởi tạo MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm = MoveGroupCommander("manipulator")
        
        # Cấu hình tốc độ an toàn
        self.arm.set_planning_time(10)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        
        rospy.sleep(2)
        rospy.loginfo("✓ UR5e Pick & Place System READY")
        
    def add_table_and_objects(self):
        """Thêm bàn và vật cản vào scene"""
        rospy.loginfo("Adding table and objects to scene...")
        
        # Xóa các vật cũ
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        rospy.sleep(1)
        
        # Thêm bàn
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.02
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", table_pose, (0.8, 1.0, 0.04))
        
        # Thêm vật cản (hộp lớn)
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = "base_link"
        obstacle_pose.pose.position.x = 0.4
        obstacle_pose.pose.position.y = 0.2
        obstacle_pose.pose.position.z = 0.1
        obstacle_pose.pose.orientation.w = 1.0
        self.scene.add_box("obstacle", obstacle_pose, (0.1, 0.1, 0.2))
        
        # Thêm đồ vật cần nhặt (hộp nhỏ)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0.05
        target_pose.pose.orientation.w = 1.0
        self.scene.add_box("target_object", target_pose, (0.05, 0.05, 0.05))
        
        rospy.sleep(2)
        rospy.loginfo("✓ Scene setup complete")
        
    def go_to_pose(self, x, y, z, description="target"):
        """Di chuyển đến vị trí xyz"""
        rospy.loginfo(f"Moving to {description}: [{x:.2f}, {y:.2f}, {z:.2f}]")
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Orientation hướng xuống
        pose.orientation.x = 0.707
        pose.orientation.y = 0.707
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        
        if success:
            rospy.loginfo(f"✓ Reached {description}")
        else:
            rospy.logwarn(f"✗ Failed to reach {description}")
            
        return success
    
    def attach_object(self, object_name):
        """Gắn vật vào gripper (giả lập pick)"""
        rospy.loginfo(f"Attaching {object_name}...")
        eef_link = self.arm.get_end_effector_link()
        self.scene.attach_box(eef_link, object_name)
        rospy.sleep(1)
        rospy.loginfo("✓ Object attached")
        
    def detach_object(self, object_name):
        """Tháo vật khỏi gripper (giả lập place)"""
        rospy.loginfo(f"Detaching {object_name}...")
        eef_link = self.arm.get_end_effector_link()
        self.scene.remove_attached_object(eef_link, object_name)
        rospy.sleep(1)
        rospy.loginfo("✓ Object detached")
        
    def pick_and_place_demo(self):
        """Demo pick & place hoàn chỉnh"""
        rospy.loginfo("="*50)
        rospy.loginfo("STARTING PICK & PLACE DEMO")
        rospy.loginfo("="*50)
        
        # Setup scene
        self.add_table_and_objects()
        
        # Bước 1: Di chuyển về home
        rospy.loginfo("\n[1/6] Moving to HOME position")
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        rospy.sleep(2)
        
        # Bước 2: Di chuyển đến trên vật cần nhặt
        rospy.loginfo("\n[2/6] Moving above target object")
        self.go_to_pose(0.5, -0.2, 0.3, "above object")
        rospy.sleep(1)
        
        # Bước 3: Hạ xuống pick
        rospy.loginfo("\n[3/6] Lowering to pick")
        self.go_to_pose(0.5, -0.2, 0.08, "pick position")
        rospy.sleep(1)
        
        # Bước 4: Attach object (giả lập pick)
        rospy.loginfo("\n[4/6] Picking object")
        self.attach_object("target_object")
        
        # Bước 5: Nâng lên
        rospy.loginfo("\n[5/6] Lifting object")
        self.go_to_pose(0.5, -0.2, 0.3, "lift position")
        rospy.sleep(1)
        
        # Bước 6: Di chuyển đến vị trí đặt (tránh vật cản)
        rospy.loginfo("\n[6/6] Moving to place location")
        self.go_to_pose(0.5, 0.3, 0.3, "above place location")
        rospy.sleep(1)
        
        # Bước 7: Hạ xuống place
        rospy.loginfo("\n[7/7] Placing object")
        self.go_to_pose(0.5, 0.3, 0.08, "place position")
        self.detach_object("target_object")
        rospy.sleep(1)
        
        # Bước 8: Quay về home
        rospy.loginfo("\n[8/8] Returning home")
        self.go_to_pose(0.5, 0.3, 0.3, "lift after place")
        rospy.sleep(1)
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("PICK & PLACE DEMO COMPLETED!")
        rospy.loginfo("="*50)


def main():
    try:
        controller = UR5ePickPlace()
        
        print("\n" + "="*60)
        print("UR5e PICK & PLACE CONTROL")
        print("="*60)
        print("Commands:")
        print("  1 - Run full pick & place demo")
        print("  2 - Just add scene objects")
        print("  q - Quit")
        print("="*60)
        
        while not rospy.is_shutdown():
            cmd = input("\n>>> Enter command: ").strip()
            
            if cmd == "1":
                controller.pick_and_place_demo()
            elif cmd == "2":
                controller.add_table_and_objects()
            elif cmd in ["q", "quit", "exit"]:
                rospy.loginfo("Shutting down...")
                break
            else:
                print("Unknown command")
                
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")


if __name__ == "__main__":
    main()
