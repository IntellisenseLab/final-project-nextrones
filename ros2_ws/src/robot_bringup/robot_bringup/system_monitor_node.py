#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from semantic_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist
import time

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # Topic tracking (Topic Name -> [Last Message Time, Total Count])
        self.topics = {
            '/camera/color/image_raw/compressed': [0.0, 0],
            '/detections': [0.0, 0],
            '/scan': [0.0, 0],
            '/cmd_vel': [0.0, 0]
        }
        
        # Subscriptions
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', 
                                 lambda msg: self.topic_callback('/camera/color/image_raw/compressed'), 10)
        self.create_subscription(DetectionArray, '/detections', 
                                 lambda msg: self.topic_callback('/detections'), 10)
        self.create_subscription(LaserScan, '/scan', 
                                 lambda msg: self.topic_callback('/scan'), 10)
        self.create_subscription(Twist, '/cmd_vel', 
                                 lambda msg: self.topic_callback('/cmd_vel'), 10)
        
        # Periodic Status Print (Every 5 seconds)
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('🛡️ System Monitor Started — Tracking topic health...')

    def topic_callback(self, topic_name):
        self.topics[topic_name][0] = time.time()
        self.topics[topic_name][1] += 1

    def print_status(self):
        now = time.time()
        self.get_logger().info('--- 📊 SYSTEM HEALTH REPORT ---')
        
        # Check Pi/Hardware Connection
        pi_active = (now - self.topics['/camera/color/image_raw/compressed'][0] < 2.0 or 
                     now - self.topics['/scan'][0] < 2.0)
        
        status_icon = '✅' if pi_active else '❌'
        status_msg = 'Hardware Connected & Streaming' if pi_active else 'Hardware Disconnected or Idle'
        self.get_logger().info(f'{status_icon} Status: {status_msg}')
        
        for topic, data in self.topics.items():
            last_time, count = data
            idle_time = now - last_time if last_time > 0 else float('inf')
            
            # Calculate FPS over the last 5 seconds
            if not hasattr(self, '_prev_counts'): self._prev_counts = {t: 0 for t in self.topics}
            msgs_since_last = count - self._prev_counts[topic]
            fps = msgs_since_last / 5.0
            self._prev_counts[topic] = count
            
            if idle_time < 2.0:
                self.get_logger().info(f'  🔹 {topic}: ACTIVE | {fps:.1f} FPS')
            else:
                self.get_logger().info(f'  🔸 {topic}: IDLE (last seen {idle_time:.1f}s ago)')
        
        self.get_logger().info('-------------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
