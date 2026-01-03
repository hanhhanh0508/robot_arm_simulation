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
        rospy.init_node("ur5e_interactive_control", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)
        
        self.arm.set_planning_time(15.0)
        self.arm.set_num_planning_attempts(20)
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_planner_id("RRTConnect")
        
        self.reference_frame = self.arm.get_planning_frame()
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        self.positions = {
            "home": [0.3, 0.0, 0.4],
            "pick": [0.4, 0.2, 0.3],
            "place": [0.4, -0.2, 0.3],
        }
        
        rospy.loginfo("UR5e READY!")
    
    def move_to_position(self, pos, label="TARGET"):
        rospy.loginfo(f"Moving to {label}...")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        q = quaternion_from_euler(-math.pi/2, 0, 0)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        
        if success:
            rospy.loginfo(f"✓ Reached {label}")
        else:
            rospy.logwarn(f"✗ Failed {label}")
        return success
    
    def create_obstacles(self, num=2):
        rospy.loginfo(f"Creating {num} obstacles...")
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        
        for i in range(num):
            x = random.uniform(0.28, 0.42)
            y = random.uniform(-0.15, 0.15)
            z = random.uniform(0.25, 0.38)
            size = random.uniform(0.06, 0.10)
            
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            
            self.scene.add_box(f"obstacle_{i}", pose, (size, size, size))
            rospy.loginfo(f"  Obstacle {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        rospy.sleep(1)
        rospy.loginfo("✓ Obstacles created")
    
    def demo(self):
        rospy.loginfo("\n=== DEMO START ===")
        
        # 1. Home
        self.move_to_position(self.positions["home"], "HOME")
        rospy.sleep(1)
        
        # 2. Create obstacles
        self.create_obstacles(2)
        
        # 3. Pick
        self.move_to_position(self.positions["pick"], "PICK")
        rospy.sleep(1)
        
        # 4. Place
        self.move_to_position(self.positions["place"], "PLACE")
        rospy.sleep(1)
        
        # 5. Home
        self.move_to_position(self.positions["home"], "HOME")
        
        rospy.loginfo("\n=== DEMO COMPLETE ===")

def main():
    try:
        controller = UR5eInteractiveControl()
        
        while not rospy.is_shutdown():
            cmd = input("\nCommands: [1=Demo, q=Quit]: ").strip()
            
            if cmd == '1':
                controller.demo()
            elif cmd in ['q', 'quit']:
                break
            else:
                print("Unknown command")
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
