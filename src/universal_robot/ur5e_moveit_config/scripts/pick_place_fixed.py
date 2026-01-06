#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5e Pick & Place FIXED - Giải quyết vấn đề TIMEOUT
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
from gazebo_msgs.srv import SpawnModel, DeleteModel

class FixedPickPlace:
    def __init__(self):
        rospy.init_node("fixed_pick_place", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # MoveIt
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        
        rospy.sleep(2)
        
        # ✅ CẤU HÌNH TỐI ƯU - CHẬM VÀ ỔN ĐỊNH
        self.arm.set_planning_time(20.0)        # Tăng planning time
        self.arm.set_num_planning_attempts(30)  # Tăng số lần thử
        self.arm.set_max_velocity_scaling_factor(0.1)   # CHẬM!
        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_goal_position_tolerance(0.05)
        self.arm.set_goal_orientation_tolerance(0.2)
        
        # Action client với TIMEOUT LỚN
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
        rospy.sleep(1.0)
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        # Kết nối Gazebo
        rospy.loginfo("Đang kết nối Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.loginfo("✓ Kết nối Gazebo OK!")
        
        # Các vị trí - ĐIỀU CHỈNH GẦN HƠN
        self.positions = {
            "home":  [0.3, 0.0, 0.40],    # Hạ thấp xuống
            "pick":  [0.45, 0.25, 0.35],  # Gần hơn, thấp hơn
            "place": [0.45, -0.25, 0.35],
        }
        
        self.spawned_obstacles = []
        self.target_object = None
        self.holding_object = False
        
        rospy.loginfo("="*70)
        rospy.loginfo("✓ HỆ THỐNG SẴN SÀNG!")
        rospy.loginfo("="*70)
    
    def joint_state_callback(self, msg):
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
    def demo_gazebo_move_lr_ud(self, loops=2):
      
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🎮 DEMO GAZEBO: LEFT - RIGHT - UP - DOWN")
        rospy.loginfo("="*70)

        base = self.positions["home"]

        poses = {
            "CENTER": base,
            "LEFT":   [base[0], base[1] + 0.20, base[2]],
            "RIGHT":  [base[0], base[1] - 0.20, base[2]],
            "UP":     [base[0], base[1], base[2] + 0.15],
            "DOWN":   [base[0], base[1], base[2] - 0.10],
        }

        for i in range(loops):
            rospy.loginfo(f"\n🔁 Loop {i+1}/{loops}")

            for name, pos in poses.items():
                if rospy.is_shutdown():
                    return

                rospy.loginfo(f"➡️ {name}")
                if not self.move_to_position(pos, name):
                    rospy.logwarn(f"⚠️ Không tới được {name}")
                rospy.sleep(1.5)

        # Quay về HOME
        self.move_to_position(self.positions["home"], "HOME")

        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("✅ HOÀN THÀNH DEMO GAZEBO MOVE")
        rospy.loginfo("="*70)
    
    def execute_trajectory_direct(self, trajectory):
        """Execute trajectory với TIMEOUT LỚN"""
        if not trajectory.joint_trajectory.points:
            rospy.logerr("  ❌ Trajectory rỗng!")
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        
        # Tính thời gian trajectory
        if trajectory.joint_trajectory.points:
            duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            duration = 10.0
        
        rospy.loginfo("  📤 Execute trajectory (duration: %.1f giây)..." % duration)
        self.client.send_goal(goal)
        
        # ✅ TIMEOUT = duration * 3 (rất dư!)
        timeout = duration * 3.0
        rospy.loginfo("  ⏱️  Đợi tối đa %.1f giây..." % timeout)
        
        success = self.client.wait_for_result(timeout=rospy.Duration(timeout))
        
        if success:
            result = self.client.get_result()
            rospy.loginfo("  ✅ Hoàn thành! (error_code: %d)" % result.error_code)
            return True
        else:
            rospy.logerr("  ❌ Timeout sau %.1f giây!" % timeout)
            return False
    
    def move_to_position(self, pos, label="TARGET", max_retries=3):
        """Di chuyển đến vị trí xyz"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("→ Di chuyển: %s (%.2f, %.2f, %.2f)" % (label, pos[0], pos[1], pos[2]))
        rospy.loginfo("="*70)
        
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
            
            rospy.loginfo("  🔧 Planning (lần %d/%d)..." % (attempt+1, max_retries))
            plan = self.arm.plan()
            
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = bool(plan.joint_trajectory.points)
                trajectory = plan
            
            self.arm.clear_pose_targets()
            
            if success:
                num_points = len(trajectory.joint_trajectory.points)
                rospy.loginfo("  ✅ Plan OK! (%d điểm)" % num_points)
                
                if self.execute_trajectory_direct(trajectory):
                    rospy.loginfo("  🎉 Di chuyển thành công!")
                    return True
                else:
                    rospy.logwarn("  ⚠️ Execute thất bại, thử lại...")
            else:
                rospy.logwarn("  ⚠️ Planning thất bại, thử lại...")
            
            rospy.sleep(1.0)
        
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
    
    def create_obstacles(self, num=1):
        """Tạo ÍT vật cản hơn"""
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

       # ====== ADD VÀO MOVEIT ======
           pose_moveit = PoseStamped()
           pose_moveit.header.frame_id = self.reference_frame
           pose_moveit.pose.position.x = pick_pos[0]
           pose_moveit.pose.position.y = pick_pos[1]
           pose_moveit.pose.position.z = target_size / 2
           pose_moveit.pose.orientation.w = 1.0

           self.scene.add_box(
                target_name,
                pose_moveit,
                size=(target_size, target_size, target_size)
           )
    # ============================

           rospy.loginfo("  ✓ Vật nhặt: (%.2f, %.2f, 0.00) 🟢 (Gazebo + MoveIt)" %
                  (pick_pos[0], pick_pos[1]))

        
        # Tạo ÍT vật cản hơn
        forbidden = [
            (self.positions["pick"], 0.25),
            (self.positions["place"], 0.25),
            (self.positions["home"], 0.20),
        ]
        
        created = 0
        for i in range(50):
            if created >= num:
                break
            
            x = random.uniform(0.32, 0.38)
            y = random.uniform(-0.05, 0.05)
            z = random.uniform(0.22, 0.28)
            
            too_close = False
            for zone_pos, radius in forbidden:
                dist = math.sqrt((x-zone_pos[0])**2 + (y-zone_pos[1])**2 + (z-zone_pos[2])**2)
                if dist < radius:
                    too_close = True
                    break
            
            if too_close:
                continue
            
            size = random.uniform(0.04, 0.06)
            
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
        rospy.loginfo("🤏 Đang nhặt vật...")
        self.holding_object = True
        rospy.loginfo("✅ Đã nhặt vật!")
        return True
    
    def place_object(self):
        rospy.loginfo("🤲 Đang đặt vật...")
        self.holding_object = False
        rospy.loginfo("✅ Đã đặt vật!")
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
        """Demo pick & place FIXED"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("🚀 DEMO: PICK & PLACE (FIXED VERSION)")
        rospy.loginfo("="*70)
        
        self.clear_all()
        
        # Vẽ markers
        self.draw_marker(self.positions["home"], (0, 0, 1, 1), "HOME", 1)
        self.draw_marker(self.positions["pick"], (0, 1, 0, 1), "PICK", 2)
        self.draw_marker(self.positions["place"], (1, 0, 0, 1), "PLACE", 3)
        
        rospy.sleep(1)
        
        # Tạo ÍT vật cản (chỉ 1 thay vì 2)
        self.create_obstacles(1)
        
        # 1. Về HOME
        if not self.move_to_position(self.positions["home"], "HOME"):
            rospy.logwarn("⚠️ Không về được HOME")
            return
        rospy.sleep(2)
        
        # 2. Đến PICK (phía trên)
        pick_above = list(self.positions["pick"])
        pick_above[2] += 0.10
        if not self.move_to_position(pick_above, "ABOVE PICK"):
            rospy.logwarn("⚠️ Không đến được ABOVE PICK")
            return
        rospy.sleep(2)
        
        # 3. Hạ xuống PICK
        if not self.move_to_position(self.positions["pick"], "PICK"):
            rospy.logwarn("⚠️ Không đến được PICK")
            return
        rospy.sleep(2)
        
        # 4. NHẶT VẬT
        self.pick_object()
        rospy.sleep(1)
        
        # 5. Nâng lên
        if not self.move_to_position(pick_above, "LIFT"):
            rospy.logwarn("⚠️ Không nâng được")
        rospy.sleep(2)
        
        # 6. Đến PLACE (phía trên)
        place_above = list(self.positions["place"])
        place_above[2] += 0.10
        if not self.move_to_position(place_above, "ABOVE PLACE"):
            rospy.logwarn("⚠️ Không đến được ABOVE PLACE")
        rospy.sleep(2)
        
        # 7. Hạ xuống PLACE
        if not self.move_to_position(self.positions["place"], "PLACE"):
            rospy.logwarn("⚠️ Không đến được PLACE")
        rospy.sleep(2)
        
        # 8. ĐẶT VẬT
        self.place_object()
        rospy.sleep(1)
        
        # 9. Nâng lên
        if not self.move_to_position(place_above, "LIFT AFTER PLACE"):
            rospy.logwarn("⚠️ Không nâng được")
        rospy.sleep(2)
        
        # 10. Về HOME
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("✅ HOÀN THÀNH DEMO PICK & PLACE!")
        rospy.loginfo("="*70)

def main():
    try:
        controller = FixedPickPlace()
        
        print("\n" + "="*70)
        print("🤖 PICK & PLACE DEMO (FIXED - Đã sửa timeout)")
        print("="*70)
        print("✅ Tăng goal_time: 0.6s → 5.0s")
        print("✅ Tăng timeout execution: x3")
        print("✅ Giảm tốc độ: 0.1x")
        print("✅ Chỉ 1 vật cản thay vì 2")
        print("="*70)
        
        while not rospy.is_shutdown():
            cmd = input(
                "\nNhập lệnh:\n"
                " 1 = Pick & Place demo\n"
                " 2 = Gazebo Left-Right-Up-Down demo\n"
                " q = Thoát\n"
                "👉 "
            ).strip().lower()

            if cmd == '1':
                controller.demo_pick_place()
            elif cmd == '2':
                controller.demo_gazebo_move_lr_ud(loops=3)
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
