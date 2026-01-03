#!/usr/bin/env python3
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
        rospy.init_node("ur5e_optimized_control", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        
        rospy.sleep(2)
        
        # ===== IMPROVED CONFIGURATION =====
        self.arm.set_planning_time(30.0)  # Tăng lên 30 giây
        self.arm.set_num_planning_attempts(50)  # Tăng lên 50 lần
        self.arm.set_max_velocity_scaling_factor(0.2)  # Giảm tốc độ
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_planner_id("RRTConnect")
        
        # Tăng tolerance
        self.arm.set_goal_position_tolerance(0.02)  # 2cm
        self.arm.set_goal_orientation_tolerance(0.15)  # ~8 độ
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        rospy.sleep(1)

        # ===== VỊ TRÍ TỐI ƯU HƠN =====
        self.positions = {
            "home":  [0.3, 0.0, 0.45],    # Cao hơn để dễ về
            "pick":  [0.50, 0.30, 0.40],  # Di xa khỏi trung tâm
            "place": [0.50, -0.30, 0.40], # Đối xứng
            "safe":  [0.25, 0.0, 0.55]    # Điểm an toàn cao
        }
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("UR5e OPTIMIZED CONTROL - READY (v2)")
        rospy.loginfo("Improved: Longer planning, safer zones, better tolerance")
        rospy.loginfo("=" * 70)

    def create_obstacles(self, num=2):
        """Tạo vật cản AN TOÀN HƠN - tránh xa các điểm quan trọng"""
        rospy.loginfo(f"Creating {num} obstacles (safer placement)...")
        
        self.clear_all_objects()
        
        # Vùng cấm tuyệt đối (không spawn ở đây)
        forbidden_zones = [
            (self.positions["pick"], 0.20),   # 20cm xung quanh PICK
            (self.positions["place"], 0.20),  # 20cm xung quanh PLACE
            (self.positions["home"], 0.18),   # 18cm xung quanh HOME
        ]
        
        created = 0
        max_attempts = 100
        attempt = 0
        
        while created < num and attempt < max_attempts:
            attempt += 1
            
            # Random trong vùng AN TOÀN
            x = random.uniform(0.28, 0.38)  # Gần trung tâm hơn
            y = random.uniform(-0.10, 0.10)  # Hẹp hơn
            z = random.uniform(0.20, 0.32)   # Thấp hơn
            
            # Check forbidden zones
            too_close = False
            for zone_pos, radius in forbidden_zones:
                dist = math.sqrt(
                    (x - zone_pos[0])**2 + 
                    (y - zone_pos[1])**2 + 
                    (z - zone_pos[2])**2
                )
                if dist < radius:
                    too_close = True
                    break
            
            if too_close:
                continue
            
            # Tạo vật cản nhỏ hơn
            size = random.uniform(0.05, 0.07)
            
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            
            name = f"obstacle_{created}"
            self.scene.add_box(name, pose, (size, size, size))
            
            rospy.loginfo(f"  ✓ Obstacle {created+1}: ({x:.2f}, {y:.2f}, {z:.2f}) size={size:.2f}")
            created += 1
        
        rospy.sleep(2.0)  # Chờ lâu hơn để scene update
        rospy.loginfo(f"✓ Created {created} obstacles (safe placement)")

    def move_to_position(self, pos, label="TARGET", max_retries=5):
        """Di chuyển với nhiều retry hơn"""
        rospy.loginfo(f"→ Moving to {label}...")
        
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        
        # Orientation nhìn xuống
        q = quaternion_from_euler(-math.pi/2, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        # Retry mechanism
        for attempt in range(max_retries):
            self.arm.set_pose_target(pose)
            
            rospy.loginfo(f"  Planning attempt {attempt + 1}/{max_retries}...")
            plan = self.arm.plan()
            
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = plan
                trajectory = plan
            
            if success:
                rospy.loginfo(f"  ✓ Plan found! Executing...")
                success = self.arm.execute(trajectory, wait=True)
                self.arm.stop()
                self.arm.clear_pose_targets()
                
                if success:
                    rospy.loginfo(f"✓ Reached {label}")
                    return True
                else:
                    rospy.logwarn(f"  Execution failed, retrying...")
            else:
                rospy.logwarn(f"  Planning failed, retrying...")
                rospy.sleep(1.0)  # Chờ lâu hơn giữa các lần thử
        
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.logerr(f"✗ Failed to reach {label} after {max_retries} attempts")
        return False

    def clear_all_objects(self):
        """Xóa objects"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        
        rospy.sleep(0.5)

    def draw_sphere_marker(self, position, color, label, marker_id):
        """Vẽ marker"""
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

    def demo_basic(self):
        """Demo cơ bản"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("DEMO: PICK & PLACE with SAFER OBSTACLES")
        rospy.loginfo("="*70)
        
        self.clear_all_objects()
        
        # Draw markers
        self.draw_sphere_marker(self.positions["home"], (0, 0, 1, 1), "HOME", 1)
        self.draw_sphere_marker(self.positions["pick"], (0, 1, 0, 1), "PICK", 2)
        self.draw_sphere_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 3)
        
        rospy.sleep(1)
        
        # Tạo vật cản AN TOÀN
        self.create_obstacles(2)
        
        # Di chuyển
        if not self.move_to_position(self.positions["home"], "HOME"):
            rospy.logwarn("Failed HOME")
            return
        rospy.sleep(2)
        
        if not self.move_to_position(self.positions["pick"], "PICK"):
            rospy.logwarn("Failed PICK, trying PLACE...")
        rospy.sleep(2)
        
        if not self.move_to_position(self.positions["place"], "PLACE"):
            rospy.logwarn("Failed PLACE")
        rospy.sleep(2)
        
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n✓ DEMO COMPLETE\n")

    def run(self):
        """Main loop"""
        print("\n" + "="*70)
        print("COMMANDS:")
        print("1 - Run demo")
        print("q - Quit")
        print("="*70)
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n>>> Command: ").strip().lower()
                
                if cmd == '1':
                    self.demo_basic()
                elif cmd in ['q', 'quit']:
                    rospy.loginfo("Shutting down...")
                    break
                else:
                    print("Invalid command")
                    
            except KeyboardInterrupt:
                break

def main():
    try:
        controller = UR5eInteractiveControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()