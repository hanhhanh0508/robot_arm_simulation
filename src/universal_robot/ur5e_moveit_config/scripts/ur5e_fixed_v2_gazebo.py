#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script có spawn vật cản vào GAZEBO + MoveIt
"""
import sys
import rospy
import moveit_commander
import actionlib
import random
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Point

def normalize_angle(angle):
    """Chuẩn hóa góc về khoảng [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class FixedPickPlaceGazebo:
    def __init__(self):
        rospy.init_node("fixed_pick_place_gazebo", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # MoveIt
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        
        rospy.sleep(2)
        
        # Cấu hình
        self.arm.set_planning_time(15.0)
        self.arm.set_num_planning_attempts(20)
        self.arm.set_max_velocity_scaling_factor(0.15)
        self.arm.set_max_acceleration_scaling_factor(0.15)
        self.arm.set_goal_position_tolerance(0.03)
        self.arm.set_goal_orientation_tolerance(0.15)
        
        # Action client
        rospy.loginfo("Kết nối controller...")
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("✗ Không tìm thấy controller!")
            return
        
        rospy.loginfo("✓ Kết nối controller OK!")
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Nhận joint states
        self.current_joints = None
        self.raw_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Đợi joint states
        rospy.loginfo("Đang đợi đủ 6 joint states...")
        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while rospy.Time.now() < timeout:
            if self.current_joints is not None and len(self.current_joints) >= 6:
                break
            rospy.sleep(0.1)
        
        if self.current_joints is None or len(self.current_joints) < 6:
            rospy.logerr("❌ Timeout: Không nhận được đủ 6 joint states!")
            return
        
        rospy.loginfo("✓ Đã nhận đủ %d joint states!" % len(self.current_joints))
        
        # Fix initial state
        self.fix_initial_state()
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        # ✅ Kết nối Gazebo services
        rospy.loginfo("Đang kết nối Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.loginfo("✓ Kết nối Gazebo services OK!")
        
        # Các vị trí
        self.positions = {
            "home":  [0.3, 0.0, 0.45],
            "pick":  [0.50, 0.30, 0.40],
            "place": [0.50, -0.30, 0.40],
        }
        
        # Lưu list vật cản đã spawn
        self.spawned_obstacles = []
        
        rospy.loginfo("="*70)
        rospy.loginfo("✓ HỆ THỐNG SẴN SÀNG!")
        rospy.loginfo("="*70)
    
    def joint_state_callback(self, msg):
        """Callback - Thread-safe"""
        temp_raw = {}
        temp_current = {}
        
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                raw_angle = msg.position[i]
                normalized_angle = normalize_angle(raw_angle)
                
                temp_raw[name] = raw_angle
                temp_current[name] = normalized_angle
        
        # Chỉ update khi có đủ 6 joints
        if len(temp_current) == 6:
            self.raw_joints = temp_raw
            self.current_joints = temp_current
    
    def fix_initial_state(self):
        """Kiểm tra và fix vị trí ban đầu"""
        if self.current_joints is None or len(self.current_joints) < 6:
            return False
        
        rospy.loginfo("\n🔍 Kiểm tra vị trí ban đầu...")
        
        invalid = False
        for name in self.joint_names:
            if name not in self.raw_joints:
                return False
            
            raw = self.raw_joints[name]
            if abs(raw) > 10.0:
                invalid = True
                break
        
        if invalid:
            rospy.logwarn("⚠️  Phát hiện góc khớp không hợp lệ! Đang fix...")
            normalized_values = [self.current_joints[name] for name in self.joint_names]
            
            if self.send_to_joint_angles(normalized_values, "Fix vị trí ban đầu", duration=2.0):
                rospy.loginfo("✅ Đã fix thành công!")
                rospy.sleep(1.0)
                return True
        else:
            rospy.loginfo("✅ Vị trí ban đầu hợp lệ")
            return True
    
    def send_to_joint_angles(self, joint_values, description="", duration=3.0):
        """Gửi trajectory trực tiếp"""
        if self.current_joints is None or len(self.current_joints) < 6:
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        point1 = JointTrajectoryPoint()
        try:
            point1.positions = [self.current_joints[name] for name in self.joint_names]
        except KeyError:
            return False
        point1.velocities = [0.0] * 6
        point1.time_from_start = rospy.Duration(0.0)
        
        point2 = JointTrajectoryPoint()
        point2.positions = joint_values
        point2.velocities = [0.0] * 6
        point2.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points = [point1, point2]
        
        self.client.send_goal(goal)
        success = self.client.wait_for_result(timeout=rospy.Duration(duration + 2.0))
        
        return success
    
    def execute_trajectory_direct(self, trajectory):
        """Execute trajectory"""
        if not trajectory.joint_trajectory.points:
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        
        rospy.loginfo("  📤 Execute...")
        self.client.send_goal(goal)
        
        if trajectory.joint_trajectory.points:
            duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            duration = 5.0
        
        success = self.client.wait_for_result(timeout=rospy.Duration(duration + 5.0))
        
        if success:
            result = self.client.get_result()
            if result.error_code == 0 or result.error_code == -4:
                rospy.loginfo("  ✅ Hoàn thành!")
                return True
        
        rospy.logerr("  ❌ Thất bại!")
        return False
    
    def move_to_position(self, pos, label="TARGET", max_retries=3):
        """Di chuyển đến vị trí xyz"""
        rospy.loginfo("\n→ Di chuyển: %s (%.2f, %.2f, %.2f)" % (label, pos[0], pos[1], pos[2]))
        
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        
        q = quaternion_from_euler(-math.pi/2, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        for attempt in range(max_retries):
            if self.current_joints is None or len(self.current_joints) < 6:
                rospy.sleep(0.5)
                continue
            
            missing = [n for n in self.joint_names if n not in self.current_joints]
            if missing:
                rospy.sleep(0.5)
                continue
            
            try:
                current_state = self.arm.get_current_state()
                positions_list = list(current_state.joint_state.position)
                
                for name in self.joint_names:
                    if name not in current_state.joint_state.name:
                        continue
                    idx = current_state.joint_state.name.index(name)
                    positions_list[idx] = self.current_joints[name]
                
                current_state.joint_state.position = positions_list
                
                self.arm.set_start_state(current_state)
                self.arm.set_pose_target(pose)
                
                rospy.loginfo("  Lần thử %d/%d..." % (attempt+1, max_retries))
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
                    if self.execute_trajectory_direct(trajectory):
                        return True
                
                rospy.logwarn("  ⚠️ Thử lại...")
                rospy.sleep(0.5)
                
            except Exception as e:
                rospy.logerr("  ❌ Exception: %s" % str(e))
                rospy.sleep(0.5)
                continue
        
        rospy.logerr("✗ Thất bại sau %d lần thử!" % max_retries)
        return False
    
    def spawn_box_in_gazebo(self, name, x, y, z, size):
        """Spawn hộp vào Gazebo"""
        box_sdf = """
        <?xml version='1.0'?>
        <sdf version='1.6'>
          <model name='%s'>
            <static>true</static>
            <link name='link'>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>%f %f %f</size>
                  </box>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <box>
                    <size>%f %f %f</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """ % (name, size, size, size, size, size, size)
        
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z + size/2  # Nâng lên để không chìm vào đất
        initial_pose.orientation.w = 1.0
        
        try:
            self.spawn_model(name, box_sdf, "", initial_pose, "world")
            return True
        except Exception as e:
            rospy.logwarn("  Không thể spawn '%s': %s" % (name, str(e)))
            return False
    
    def create_obstacles(self, num=2):
        """Tạo vật cản trong MoveIt VÀ Gazebo"""
        rospy.loginfo("\nTạo %d vật cản..." % num)
        
        # Xóa vật cản cũ trong MoveIt
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        # Xóa vật cản cũ trong Gazebo
        for obstacle_name in self.spawned_obstacles:
            try:
                self.delete_model(obstacle_name)
            except:
                pass
        self.spawned_obstacles = []
        
        rospy.sleep(0.5)
        
        # Vùng cấm
        forbidden = [
            (self.positions["pick"], 0.20),
            (self.positions["place"], 0.20),
            (self.positions["home"], 0.18),
        ]
        
        created = 0
        for i in range(50):
            if created >= num:
                break
            
            x = random.uniform(0.30, 0.40)
            y = random.uniform(-0.08, 0.08)
            z = random.uniform(0.20, 0.30)
            
            too_close = False
            for zone_pos, radius in forbidden:
                dist = math.sqrt((x-zone_pos[0])**2 + (y-zone_pos[1])**2 + (z-zone_pos[2])**2)
                if dist < radius:
                    too_close = True
                    break
            
            if too_close:
                continue
            
            size = random.uniform(0.05, 0.07)
            
            # ✅ Thêm vào MoveIt Planning Scene
            pose_moveit = PoseStamped()
            pose_moveit.header.frame_id = self.reference_frame
            pose_moveit.pose.position.x = x
            pose_moveit.pose.position.y = y
            pose_moveit.pose.position.z = z
            pose_moveit.pose.orientation.w = 1.0
            
            name = "obstacle_%d" % created
            self.scene.add_box(name, pose_moveit, (size, size, size))
            
            # ✅ Spawn vào Gazebo
            if self.spawn_box_in_gazebo(name, x, y, z, size):
                self.spawned_obstacles.append(name)
                rospy.loginfo("  ✓ Vật cản %d: (%.2f, %.2f, %.2f) size=%.2f [MoveIt + Gazebo]" % 
                             (created+1, x, y, z, size))
                created += 1
        
        rospy.sleep(1.5)
        rospy.loginfo("✓ Đã tạo %d vật cản (trong cả MoveIt và Gazebo)" % created)
    
    def draw_marker(self, position, color, label, marker_id):
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
    
    def clear_all(self):
        """Xóa tất cả"""
        # Xóa trong MoveIt
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        # Xóa trong Gazebo
        for obstacle_name in self.spawned_obstacles:
            try:
                self.delete_model(obstacle_name)
            except:
                pass
        self.spawned_obstacles = []
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        rospy.sleep(0.5)
    
    def demo_pick_place(self):
        """Demo pick & place"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🚀 DEMO: PICK & PLACE VỚI VẬT CẢN TRONG GAZEBO")
        rospy.loginfo("="*70)
        
        self.clear_all()
        
        # Vẽ markers
        self.draw_marker(self.positions["home"], (0, 0, 1, 1), "HOME", 1)
        self.draw_marker(self.positions["pick"], (0, 1, 0, 1), "PICK", 2)
        self.draw_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 3)
        
        rospy.sleep(1)
        
        # Tạo vật cản (trong cả MoveIt VÀ Gazebo)
        self.create_obstacles(2)
        
        # Di chuyển
        if not self.move_to_position(self.positions["home"], "HOME"):
            rospy.logwarn("⚠️ Không về được HOME")
            return
        rospy.sleep(1)
        
        if not self.move_to_position(self.positions["pick"], "PICK"):
            rospy.logwarn("⚠️ Không đến được PICK")
        rospy.sleep(1)
        
        if not self.move_to_position(self.positions["place"], "PLACE"):
            rospy.logwarn("⚠️ Không đến được PLACE")
        rospy.sleep(1)
        
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("✅ HOÀN THÀNH DEMO!")
        rospy.loginfo("="*70)

def main():
    try:
        controller = FixedPickPlaceGazebo()
        
        print("\n" + "="*70)
        print("🤖 PICK & PLACE VỚI VẬT CẢN TRONG GAZEBO")
        print("="*70)
        print("✅ Vật cản xuất hiện trong cả MoveIt VÀ Gazebo")
        print("✅ Màu đỏ 🔴 dễ nhìn trong Gazebo")
        print("="*70)
        
        while not rospy.is_shutdown():
            cmd = input("\nNhập lệnh (1=Chạy demo, q=Thoát): ").strip().lower()
            
            if cmd == '1':
                controller.demo_pick_place()
            elif cmd in ['q', 'quit']:
                rospy.loginfo("👋 Thoát chương trình")
                # Dọn dẹp trước khi thoát
                controller.clear_all()
                break
            else:
                print("❌ Lệnh không hợp lệ!")
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\n👋 Thoát chương trình")

if __name__ == "__main__":
    main()
