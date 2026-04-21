#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from semantic_msgs.msg import Detection
from geometry_msgs.msg import PoseStamped
import time

class NavGoalSenderNode(Node):
    def __init__(self):
        super().__init__('nav_goal_sender_node')
        
        # Parameters
        self.declare_parameter('target_object', 'bottle')
        self.target_object = self.get_parameter('target_object').get_parameter_value().string_value
        
        # Subscriptions
        # Listens to 3D localized object positions in the 'map' frame
        self.subscription = self.create_subscription(
            Detection, 
            '/object_locations', 
            self.object_callback, 
            10
        )
        
        # Publishers
        # Sends goal to Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # State
        self.last_goal_sent_time = 0
        self.goal_cooldown = 10.0 # seconds before sending same goal again
        
        self.get_logger().info(f'✅ Nav Goal Sender Node Started. Current Target: {self.target_object}')

    def object_callback(self, msg):
        label = msg.label
        
        if label.lower() == self.target_object.lower():
            current_time = time.time()
            
            # Cooldown logic to prevent overwhelming Nav2
            if current_time - self.last_goal_sent_time > self.goal_cooldown:
                self.get_logger().info(f'🎯 Target found: {label}. Sending goal to Nav2!')
                
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.header.frame_id = 'map'
                
                # Use the localized 3D position
                goal_msg.pose.position = msg.position
                
                # Orientation is neutral for now (robot faces "forward" as defined by the path)
                goal_msg.pose.orientation.w = 1.0
                
                self.goal_pub.publish(goal_msg)
                self.last_goal_sent_time = current_time
            else:
                self.get_logger().info(f'📍 Target {label} updated, but in cooldown.')

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
