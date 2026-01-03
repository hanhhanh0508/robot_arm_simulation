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
        
        # ===== IMPROVED PLANNING CONFIGURATION =====
        self.arm.set_planning_time(20.0)  # Tăng từ 15→20 giây
        self.arm.set_num_planning_attempts(30)  # Tăng từ 20→30 lần
        self.arm.set_max_velocity_scaling_factor(0.3)  # Tăng tốc độ
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_planner_id("RRTConnect")
        
        # Thêm tolerance để dễ reach target hơn
        self.arm.set_goal_position_tolerance(0.01)  # 1cm
        self.arm.set_goal_orientation_tolerance(0.1)  # ~5 độ
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        rospy.sleep(1)

        # Định nghĩa vị trí (đã tối ưu để tránh xung đột)
        self.positions = {
            "home": [0.3, 0.0, 0.4],
            "pick": [0.45, 0.25, 0.35],  # Di xa hơn để tránh vật cản
            "place": [0.45, -0.25, 0.35],
            "safe": [0.25, 0.0, 0.5]
        }
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("UR5e OPTIMIZED CONTROL - READY")
        rospy.loginfo("Improvements: Longer planning time, better obstacle placement")
        rospy.loginfo("=" * 70)

    def clear_all_objects(self):
        """Xóa tất cả objects trong scene"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        
        rospy.sleep(0.5)
        rospy.loginfo("✓ Cleared all objects")

    def draw_sphere_marker(self, position, color, label, marker_id):
        """Vẽ sphere marker"""
        # Sphere
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
        
        # Text
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
    
    def move_to_position(self, pos, label="TARGET", max_retries=3):
        """
        Di chuyển đến vị trí với retry mechanism
        """
        rospy.loginfo(f"→ Moving to {label}...")
        
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
        
        # Retry mechanism
        for attempt in range(max_retries):
            self.arm.set_pose_target(pose)
            
            # Planning
            rospy.loginfo(f"  Planning attempt {attempt + 1}/{max_retries}...")
            plan = self.arm.plan()
            
            # Check if plan is successful (ROS Noetic returns tuple)
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
                rospy.sleep(0.5)
        
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.logerr(f"✗ Failed to reach {label} after {max_retries} attempts")
        return False

    def create_obstacles(self, num=2):
        """
        Tạo vật cản với vị trí được tối ưu
        Tránh đặt quá gần PICK/PLACE points
        """
        rospy.loginfo(f"Creating {num} obstacles (optimized placement)...")
        
        self.clear_all_objects()
        
        # Vùng an toàn xung quanh PICK/PLACE (không spawn obstacle)
        safe_zones = [
            (self.positions["pick"], 0.15),   # 15cm xung quanh PICK
            (self.positions["place"], 0.15),  # 15cm xung quanh PLACE
            (self.positions["home"], 0.15)    # 15cm xung quanh HOME
        ]
        
        created = 0
        max_attempts = 50
        attempt = 0
        
        while created < num and attempt < max_attempts:
            attempt += 1
            
            # Random position
            x = random.uniform(0.30, 0.42)
            y = random.uniform(-0.15, 0.15)
            z = random.uniform(0.22, 0.35)
            
            # Check if too close to safe zones
            too_close = False
            for safe_pos, radius in safe_zones:
                dist = math.sqrt(
                    (x - safe_pos[0])**2 + 
                    (y - safe_pos[1])**2 + 
                    (z - safe_pos[2])**2
                )
                if dist < radius:
                    too_close = True
                    break
            
            if too_close:
                continue
            
            # Create obstacle
            size = random.uniform(0.06, 0.09)
            
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
        
        rospy.sleep(1.5)
        rospy.loginfo(f"✓ Created {created} obstacles (safe placement)")

    def demo_basic(self):
        """Demo cơ bản: HOME → PICK → PLACE → HOME"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("DEMO 1: BASIC PICK & PLACE")
        rospy.loginfo("="*70)
        
        self.clear_all_objects()
        
        # Draw markers
        self.draw_sphere_marker(self.positions["home"], (0, 0, 1, 1), "HOME", 1)
        self.draw_sphere_marker(self.positions["pick"], (0, 1, 0, 1), "PICK", 2)
        self.draw_sphere_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 3)
        
        # Create obstacles
        self.create_obstacles(2)
        
        # Execute motion sequence
        if not self.move_to_position(self.positions["home"], "HOME"):
            rospy.logwarn("Failed HOME, aborting demo")
            return
        rospy.sleep(1)
        
        if not self.move_to_position(self.positions["pick"], "PICK"):
            rospy.logwarn("Failed PICK, trying PLACE anyway...")
        rospy.sleep(1)
        
        if not self.move_to_position(self.positions["place"], "PLACE"):
            rospy.logwarn("Failed PLACE, returning HOME...")
        rospy.sleep(1)
        
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n✓ DEMO COMPLETE\n")

    def demo_advanced(self):
        """Demo nâng cao: Nhiều điểm + nhiều vật cản"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("DEMO 2: ADVANCED - MULTIPLE POINTS")
        rospy.loginfo("="*70)
        
        self.create_obstacles(3)
        
        # Random waypoints
        waypoints = []
        for i in range(4):
            x = random.uniform(0.30, 0.45)
            y = random.uniform(-0.2, 0.2)
            z = random.uniform(0.32, 0.45)
            waypoints.append([x, y, z])
            self.draw_sphere_marker([x, y, z], (1, 1, 0, 1), f"P{i+1}", i+10)
        
        # Execute
        self.move_to_position(self.positions["home"], "HOME")
        rospy.sleep(0.5)
        
        for i, wp in enumerate(waypoints):
            if self.move_to_position(wp, f"Point {i+1}"):
                rospy.sleep(0.5)
            else:
                rospy.logwarn(f"Skipping point {i+1}")
        
        self.move_to_position(self.positions["home"], "HOME")
        rospy.loginfo("\n✓ ADVANCED DEMO COMPLETE\n")

    def demo_stress_test(self):
        """Demo stress test: Nhiều vật cản"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("DEMO 3: STRESS TEST - MANY OBSTACLES")
        rospy.loginfo("="*70)
        
        self.create_obstacles(5)
        
        # Simple back and forth
        points = [
            (self.positions["home"], "HOME"),
            ([0.40, 0.20, 0.40], "CORNER 1"),
            ([0.40, -0.20, 0.40], "CORNER 2"),
            (self.positions["home"], "HOME")
        ]
        
        for pos, label in points:
            if self.move_to_position(pos, label):
                rospy.sleep(0.5)
        
        rospy.loginfo("\n✓ STRESS TEST COMPLETE\n")

    def show_menu(self):
        """Menu điều khiển"""
        print("\n" + "="*70)
        print("UR5e OPTIMIZED CONTROL - MENU")
        print("="*70)
        print("1. Demo Basic (HOME → PICK → PLACE → HOME)")
        print("2. Demo Advanced (Multiple random points)")
        print("3. Demo Stress Test (Many obstacles)")
        print("4. Go HOME only")
        print("5. Clear all obstacles")
        print("-"*70)
        print("0. Settings (adjust planning time)")
        print("q. Quit")
        print("="*70)

    def adjust_settings(self):
        """Điều chỉnh cài đặt"""
        print("\nCurrent Settings:")
        print(f"  Planning time: {self.arm.get_planning_time():.1f}s")
        print(f"  Planning attempts: {self.arm.get_num_planning_attempts()}")
        print(f"  Velocity scaling: {self.arm.get_max_velocity_scaling_factor():.2f}")
        
        try:
            time_input = input("\nNew planning time (Enter to skip): ").strip()
            if time_input:
                new_time = float(time_input)
                self.arm.set_planning_time(new_time)
                rospy.loginfo(f"✓ Planning time set to {new_time}s")
            
            attempts_input = input("New planning attempts (Enter to skip): ").strip()
            if attempts_input:
                new_attempts = int(attempts_input)
                self.arm.set_num_planning_attempts(new_attempts)
                rospy.loginfo(f"✓ Planning attempts set to {new_attempts}")
        except ValueError:
            print("Invalid input")

    def run(self):
        """Main loop"""
        self.show_menu()
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n>>> Command: ").strip().lower()
                
                if cmd == '1':
                    self.demo_basic()
                    
                elif cmd == '2':
                    self.demo_advanced()
                    
                elif cmd == '3':
                    self.demo_stress_test()
                    
                elif cmd == '4':
                    self.move_to_position(self.positions["home"], "HOME")
                    
                elif cmd == '5':
                    self.clear_all_objects()
                    
                elif cmd == '0':
                    self.adjust_settings()
                    
                elif cmd in ['q', 'quit', 'exit']:
                    rospy.loginfo("Shutting down...")
                    break
                    
                else:
                    print("⚠ Invalid command")
                    
            except KeyboardInterrupt:
                rospy.loginfo("\n\nStopped by user")
                break
            except Exception as e:
                rospy.logerr(f"Error: {e}")


def main():
    try:
        controller = UR5eInteractiveControl()
        controller.run()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\nExited")

