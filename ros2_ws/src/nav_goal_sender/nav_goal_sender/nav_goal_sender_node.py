#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from semantic_msgs.msg import Detection
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import time
import math

class NavGoalSenderNode(Node):
    def __init__(self):
        super().__init__('nav_goal_sender_node')
        
        # Parameters
        self.declare_parameter('target_object', 'bottle')
        self.declare_parameter('offset_distance', 0.6) # Meters to stop before target
        
        self.target_object = self.get_parameter('target_object').get_parameter_value().string_value
        self.offset_dist = self.get_parameter('offset_distance').get_parameter_value().double_value
        
        # TF2 Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.subscription = self.create_subscription(
            Detection, '/object_locations', self.object_callback, 10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # State
        self.last_goal_pose = None
        self.last_goal_time = 0
        self.goal_cooldown = 15.0 # Seconds before re-targeting same area
        
        self.get_logger().info(f'✅ Nav Goal Sender Ready. Targeting: {self.target_object} (Offset: {self.offset_dist}m)')

    def object_callback(self, msg):
        if msg.label.lower() != self.target_object.lower():
            return

        now = time.time()
        if now - self.last_goal_time < self.goal_cooldown:
            return

        try:
            # 1. Get current robot pose in map frame
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y

            # 2. Calculate vector from target to robot
            dx = robot_x - msg.position.x
            dy = robot_y - msg.position.y
            dist = (dx**2 + dy**2)**0.5

            if dist < self.offset_dist:
                self.get_logger().info(f'🏁 Robot is already near target {msg.label}!')
                return

            # 3. Calculate offset point (safe approach)
            # We move 'offset_dist' away from the target towards the robot
            scale = self.offset_dist / dist
            goal_x = msg.position.x + (dx * scale)
            goal_y = msg.position.y + (dy * scale)

            # 4. Publish Goal
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            
            # Point towards target
            angle = math.atan2(-dy, -dx)
            goal.pose.orientation.z = math.sin(angle/2)
            goal.pose.orientation.w = math.cos(angle/2)

            self.get_logger().info(f'🎯 Mission: Navigating to {msg.label} at ({goal_x:.2f}, {goal_y:.2f})')
            self.goal_pub.publish(goal)
            self.last_goal_time = now

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'Waiting for map -> base_link TF: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ NavGoal Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
