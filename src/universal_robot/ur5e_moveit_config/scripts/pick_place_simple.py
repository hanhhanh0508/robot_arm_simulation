#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5e Pick & Place DEMO - KHÔNG cần Link Attacher
Chỉ simulate attach/detach bằng cách lưu trạng thái
"""
import sys
import rospy
import moveit_commander
import actionlib
import random
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState

def normalize_angle(angle):
    """Chuẩn hóa góc về [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class SimplePickPlace:
    def __init__(self):
        rospy.init_node("simple_pick_place", anonymous=True)
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
        
        self.current_joints = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.sleep(1.0)
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        # Kết nối Gazebo services
        rospy.loginfo("Đang kết nối Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.loginfo("✓ Kết nối Gazebo OK!")
        
        # Các vị trí
        self.positions = {
            "home":  [0.3, 0.0, 0.45],
            "pick":  [0.50, 0.30, 0.12],  # Hạ thấp để "chạm" vật
            "place": [0.50, -0.30, 0.12],
        }
        
        # Lưu list vật
        self.spawned_obstacles = []
        self.target_object = None
        self.holding_object = False  # Trạng thái đang cầm vật
        
        rospy.loginfo("="*70)
        rospy.loginfo("✓ HỆ THỐNG SẴN SÀNG!")
        rospy.loginfo("="*70)
    
    def joint_state_callback(self, msg):
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
    
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
        
        rospy.logerr("✗ Thất bại sau %d lần thử!" % max_retries)
        return False
    
    def spawn_box_in_gazebo(self, name, x, y, z, size, color="red"):
        """Spawn hộp vào Gazebo"""
        if color == "green":
            ambient = "0 1 0 1"
            diffuse = "0 1 0 1"
        else:
            ambient = "1 0 0 1"
            diffuse = "1 0 0 1"
            
        box_sdf = """
        <?xml version='1.0'?>
        <sdf version='1.6'>
          <model name='%s'>
            <static>false</static>
            <link name='link'>
              <inertial>
                <mass>0.1</mass>
              </inertial>
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
                  <ambient>%s</ambient>
                  <diffuse>%s</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """ % (name, size, size, size, size, size, size, ambient, diffuse)
        
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z + size/2
        initial_pose.orientation.w = 1.0
        
        try:
            self.spawn_model(name, box_sdf, "", initial_pose, "world")
            return True
        except Exception as e:
            rospy.logwarn("  Không thể spawn '%s': %s" % (name, str(e)))
            return False
    
    def create_obstacles(self, num=2):
        """Tạo vật cản + 1 vật để nhặt"""
        rospy.loginfo("\nTạo %d vật cản + 1 vật nhặt..." % num)
        
        # Xóa cũ
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        for obstacle_name in self.spawned_obstacles:
            try:
                self.delete_model(obstacle_name)
            except:
                pass
        if self.target_object:
            try:
                self.delete_model(self.target_object)
            except:
                pass
        
        self.spawned_obstacles = []
        self.target_object = None
        self.holding_object = False
        
        rospy.sleep(0.5)
        
        # Spawn vật cần nhặt (màu xanh)
        target_name = "target_box"
        pick_pos = self.positions["pick"]
        target_size = 0.05
        
        if self.spawn_box_in_gazebo(target_name, pick_pos[0], pick_pos[1], 
                                     0, target_size, "green"):
            self.target_object = target_name
            rospy.loginfo("  ✓ Vật nhặt: (%.2f, %.2f, 0.00) 🟢" % 
                         (pick_pos[0], pick_pos[1]))
        
        # Tạo vật cản đỏ
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
            
            # MoveIt
            pose_moveit = PoseStamped()
            pose_moveit.header.frame_id = self.reference_frame
            pose_moveit.pose.position.x = x
            pose_moveit.pose.position.y = y
            pose_moveit.pose.position.z = z
            pose_moveit.pose.orientation.w = 1.0
            
            name = "obstacle_%d" % created
            self.scene.add_box(name, pose_moveit, (size, size, size))
            
            # Gazebo
            if self.spawn_box_in_gazebo(name, x, y, z, size, "red"):
                self.spawned_obstacles.append(name)
                rospy.loginfo("  ✓ Vật cản %d: (%.2f, %.2f, %.2f) 🔴" % 
                             (created+1, x, y, z))
                created += 1
        
        rospy.sleep(1.5)
        rospy.loginfo("✓ Đã tạo %d vật cản + 1 vật nhặt" % created)
    
    def pick_object(self):
        """Giả lập nhặt vật"""
        if not self.target_object:
            rospy.logwarn("⚠️ Không có vật để nhặt!")
            return False
        
        rospy.loginfo("🤏 Đang nhặt vật '%s'..." % self.target_object)
        self.holding_object = True
        rospy.loginfo("✅ Đã nhặt vật! (giả lập)")
        return True
    
    def place_object(self):
        """Giả lập đặt vật"""
        if not self.holding_object:
            rospy.logwarn("⚠️ Không có vật đang cầm!")
            return False
        
        rospy.loginfo("🤲 Đang đặt vật...")
        self.holding_object = False
        rospy.loginfo("✅ Đã đặt vật! (giả lập)")
        return True
    
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
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        for obstacle_name in self.spawned_obstacles:
            try:
                self.delete_model(obstacle_name)
            except:
                pass
        if self.target_object:
            try:
                self.delete_model(self.target_object)
            except:
                pass
        
        self.spawned_obstacles = []
        self.target_object = None
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        rospy.sleep(0.5)
    
    def demo_pick_place(self):
        """Demo pick & place"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🚀 DEMO: PICK & PLACE (SIMULATION)")
        rospy.loginfo("="*70)
        
        self.clear_all()
        
        # Vẽ markers
        self.draw_marker(self.positions["home"], (0, 0, 1, 1), "HOME", 1)
        self.draw_marker(self.positions["pick"], (0, 1, 0, 1), "PICK", 2)
        self.draw_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 3)
        
        rospy.sleep(1)
        
        # Tạo scene
        self.create_obstacles(2)
        
        # 1. Về HOME
        if not self.move_to_position(self.positions["home"], "HOME"):
            rospy.logwarn("⚠️ Không về được HOME")
            return
        rospy.sleep(1)
        
        # 2. Đến PICK (phía trên)
        pick_above = list(self.positions["pick"])
        pick_above[2] += 0.15
        if not self.move_to_position(pick_above, "ABOVE PICK"):
            rospy.logwarn("⚠️ Không đến được ABOVE PICK")
            return
        rospy.sleep(1)
        
        # 3. Hạ xuống PICK
        if not self.move_to_position(self.positions["pick"], "PICK"):
            rospy.logwarn("⚠️ Không đến được PICK")
            return
        rospy.sleep(1)
        
        # 4. NHẶT VẬT (giả lập)
        self.pick_object()
        rospy.sleep(1)
        
        # 5. Nâng lên
        if not self.move_to_position(pick_above, "LIFT"):
            rospy.logwarn("⚠️ Không nâng được")
        rospy.sleep(1)
        
        # 6. Đến PLACE (phía trên)
        place_above = list(self.positions["place"])
        place_above[2] += 0.15
        if not self.move_to_position(place_above, "ABOVE PLACE"):
            rospy.logwarn("⚠️ Không đến được ABOVE PLACE")
        rospy.sleep(1)
        
        # 7. Hạ xuống PLACE
        if not self.move_to_position(self.positions["place"], "PLACE"):
            rospy.logwarn("⚠️ Không đến được PLACE")
        rospy.sleep(1)
        
        # 8. ĐẶT VẬT (giả lập)
        self.place_object()
        rospy.sleep(1)
        
        # 9. Nâng lên
        if not self.move_to_position(place_above, "LIFT AFTER PLACE"):
            rospy.logwarn("⚠️ Không nâng được")
        rospy.sleep(1)
        
        # 10. Về HOME
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("✅ HOÀN THÀNH DEMO PICK & PLACE!")
        rospy.loginfo("="*70)

def main():
    try:
        controller = SimplePickPlace()
        
        print("\n" + "="*70)
        print("🤖 PICK & PLACE DEMO (SIMULATION)")
        print("="*70)
        print("ℹ️  Không cần Link Attacher")
        print("✅ Vật XANH 🟢 = vật nhặt")
        print("✅ Vật ĐỎ 🔴 = vật cản")
        print("="*70)
        
        while not rospy.is_shutdown():
            cmd = input("\nNhập lệnh (1=Chạy demo, q=Thoát): ").strip().lower()
            
            if cmd == '1':
                controller.demo_pick_place()
            elif cmd in ['q', 'quit']:
                rospy.loginfo("👋 Thoát chương trình")
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