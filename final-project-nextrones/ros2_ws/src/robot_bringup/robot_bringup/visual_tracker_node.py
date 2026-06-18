#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from semantic_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist
import time

class SimpleVisualTracker(Node):
    def __init__(self):
        super().__init__('simple_visual_tracker')
        
        # Add Parameters
        self.declare_parameter('target_label', 'bottle')
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo_detections',
            self.detection_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.image_width = 640
        self.center_x = self.image_width / 2.0

        
        # Expert PID Control Gains
        self.linear_speed = 0.22 # m/s
        self.kp = 0.005 # Proportional
        self.ki = 0.0001 # Integral
        self.kd = 0.001 # Derivative
        
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()


        
        self.last_detection_time = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'🚀 Simple Visual Tracker Started. Target: {self.target_label}')

    def detection_callback(self, msg):
        for det in msg.detections:
            # 1. Target Validation
            label = det.label.lower()
            is_target = False
            if self.target_label == 'bottle':
                if label in ['bottle', 'cup', 'vase']:
                    is_target = True
            elif label == self.target_label.lower():
                is_target = True

            if is_target:
                # 2. Extract Data
                u = det.position.x
                area = det.size.z
                twist = Twist()
                
                # 3. 🔹 Proportional Steering (with Clamping & Smoothing)
                alpha = 0.3
                self.smoothed_u = (1.0 - alpha) * self.smoothed_u + alpha * u if hasattr(self, 'smoothed_u') else u
                error_x = self.center_x - self.smoothed_u
                self.last_error_x = error_x # 🧠 Memory: Which way were we turning?
                
                # 4. Expert PID Angular Control
                dt = time.time() - self.last_time
                if dt <= 0: dt = 0.033 # Fallback for high-speed frames
                
                self.integral_error += error_x * dt
                self.integral_error = max(min(self.integral_error, 50.0), -50.0) # Anti-windup
                
                derivative = (error_x - self.previous_error) / dt
                
                # PID Sum
                p_term = self.kp * error_x
                i_term = self.ki * self.integral_error
                d_term = self.kd * derivative
                
                target_angular = p_term + i_term + d_term
                
                # Rigid Clamping for stability at low FPS
                max_ang = 0.15 
                twist.angular.z = max(min(target_angular, max_ang), -max_ang)
                
                # Update states
                self.previous_error = error_x
                self.last_time = time.time()

                
                # 5. Elite Alignment & Depth-Stopping Guard
                abs_error = abs(error_x)
                
                # Safety Guard: Stop immediately if too close (Depth-Stopping)
                if area > 140000:
                    twist.linear.x = 0.0
                    if not hasattr(self, '_last_reached_log'): self._last_reached_log = 0
                    if time.time() - self._last_reached_log > 3.0:
                        self.get_logger().info('🏁 TARGET REACHED: Stopping for safety!')
                        self._last_reached_log = time.time()
                else:
                    # Constant Forward Bias if within 120px of center
                    if abs_error < 120:
                        twist.linear.x = 0.15 # Confident approach
                    else:
                        twist.linear.x = 0.05 # Forward momentum while rotating

                    
                self.publisher.publish(twist)
                
                # 🔹 Diagnostic: Log acquisition occasionally
                if not hasattr(self, '_last_acq_log'): self._last_acq_log = 0
                if time.time() - self._last_acq_log > 5.0:
                    self.get_logger().info(f'🎯 TARGET LOCKED: {label} (Dist: {area:.0f}, Err: {abs_error:.0f}px)')
                    self._last_acq_log = time.time()

                self.last_detection_time = time.time()
                break # Mission focus: follow the first valid target seen

            else:
                # 6. Diagnostic: Log rejected label occasionally
                if not hasattr(self, '_last_reject_log'): self._last_reject_log = 0
                if time.time() - self._last_reject_log > 3.0:
                    self.get_logger().info(f'📦 Ignoring {label} (Looking for {self.target_label})')
                    self._last_reject_log = time.time()



    def timer_callback(self):
        # 3. Intelligent Search Pattern (For Static Objects)
        elapsed = time.time() - self.last_detection_time
        
        if elapsed > 3.0:
            twist = Twist()
            last_err = getattr(self, 'last_error_x', 0)
            
            if elapsed < 6.0:
                # 🔄 PHASE 1: Precise Counter-Rotation Recovery
                # If target lost, rotate back slightly to re-center it
                direction = 1.0 if last_err > 0 else -1.0
                twist.angular.z = direction * 0.12 
                if not hasattr(self, '_last_search_log'): self._last_search_log = 0
                if time.time() - self._last_search_log > 2.0:
                    self.get_logger().info('� Recovery: Counter-rotating to last seen spot...')
                    self._last_search_log = time.time()
            elif elapsed < 16.0:
                # 📡 PHASE 2: Oscillatory Scanning (Left-Right)
                import math
                scan_speed = 0.15 * math.sin((elapsed - 6.0) * 1.5)
                twist.angular.z = scan_speed
                if not hasattr(self, '_last_search_log'): self._last_search_log = 0
                if time.time() - self._last_search_log > 3.0:
                    self.get_logger().info('📡 Scanning: Searching Left & Right...')
                    self._last_search_log = time.time()
            else:
                twist.angular.z = 0.0
                if self.last_detection_time > 0:
                    self.get_logger().warn(f'❌ Mission Failed: {self.target_label} Lost permanently')
            
            self.publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = SimpleVisualTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
