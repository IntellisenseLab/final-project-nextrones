#!/usr/bin/env python3
"""
Task Manager Node: The "Brain" of the autonomous robot.
- Subscribes to: /yolo/detections (String: "label:dist, label:dist")
- Subscribes to: /odom (to know current pose)
- Actions: Sends goals to Nav2 via Simple Action Client.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import time

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Parameters
        self.declare_parameter('target_object', 'bottle')
        self.declare_parameter('stop_distance', 0.8) # Stop 0.8m from object
        
        self.target_label = self.get_parameter('target_object').value
        self.stop_dist = self.get_parameter('stop_distance').value
        
        # State
        self.current_odom = None
        self.is_navigating = False
        self.last_goal_time = 0
        
        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.det_sub = self.create_subscription(String, '/yolo/detections', self.det_cb, 10)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info(f"🧠 Task Manager initialized. Searching for: '{self.target_label}'")

    def odom_cb(self, msg):
        self.current_odom = msg

    def det_cb(self, msg):
        """Parse detections and decide if we should move."""
        if self.is_navigating or self.current_odom is None:
            return

        # Simple parsing of "label:0.00m, label:0.00m"
        detections = msg.data.split(", ")
        for det in detections:
            if ":" not in det: continue
            label, dist_str = det.split(":")
            dist = float(dist_str.replace("m", ""))
            
            if label.lower() == self.target_label.lower() and dist > 0:
                self.get_logger().info(f"🎯 Target spotted! {label} at {dist:.2f}m. Planning approach...")
                self.send_goal(dist)
                break

    def send_goal(self, dist_to_obj):
        """Calculate a goal point in front of the robot and send to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        # Simple logic: Move forward by (dist_to_obj - stop_dist)
        # In a real robot, we'd use TF to transform camera->map, but 
        # for a simple demo, we'll project from current Odom.
        
        move_dist = dist_to_obj - self.stop_dist
        if move_dist < 0.1:
            self.get_logger().info("Already close enough! Mission complete.")
            return

        self.is_navigating = True
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "odom"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Current Position
        cp = self.current_odom.pose.pose.position
        co = self.current_odom.pose.pose.orientation
        
        # Simple heading projection (approximation)
        # Note: In a complete system, we use tf2_geometry_msgs to project properly.
        yaw = self.getYaw(co)
        goal_msg.pose.pose.position.x = cp.x + (move_dist * math.cos(yaw))
        goal_msg.pose.pose.position.y = cp.y + (move_dist * math.sin(yaw))
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = co # Keep same orientation
        
        self.get_logger().info(f"🚀 Sending Nav2 Goal: Moving {move_dist:.2f}m forward.")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def getYaw(self, q):
        """Simplified quaternion to yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by Nav2.")
            self.is_navigating = False
            return

        self.get_logger().info("Goal accepted. Robot is moving...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Target reached! Waiting for next detection.")
        self.is_navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
