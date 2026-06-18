#!/usr/bin/env python3
"""
ROS 2 YOLOv8 3D Object Detection Node for Jazzy.
Subscribes: 
  - /camera/color/image_raw
  - /camera/depth/image_raw
Publishes: 
  - /yolo/detections (String for now, will move to vision_msgs)
  - /yolo/object_pose (Optional GoalPose)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import message_filters

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class Yolo3DNode(Node):
    def __init__(self):
        super().__init__('yolo_3d_node')
        
        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('fps_limit', 5.0) 
        
        model_name = self.get_parameter('model').value
        self.conf = self.get_parameter('conf').value
        self.fps_limit = self.get_parameter('fps_limit').value
        self.last_ts = 0.0
        
        if YOLO is None:
            self.get_logger().error("YOLO not installed! Run: pip install ultralytics")
            return

        self.get_logger().info(f"Loading YOLO model: {model_name}...")
        self.model = YOLO(model_name)
        self.bridge = CvBridge()
        
        # Synchronized Subscriptions (RGB + Depth)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Publishers
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        
        self.get_logger().info("✅ YOLO 3D Node Ready.")

    def sync_callback(self, rgb_msg, depth_msg):
        """Processes RGB and Depth frames together."""
        now = time.time()
        if now - self.last_ts < (1.0 / self.fps_limit):
            return
        self.last_ts = now
        
        try:
            # Convert images
            color_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1') # mm
            
            # YOLO Inference
            results = self.model(color_img, conf=self.conf, verbose=False)
            
            detections = []
            for r in results:
                for box in r.boxes:
                    # Get Bounding Box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    name = self.model.names[cls]
                    
                    # Calculate depth in the center of the box
                    # We take a small crop from the center to avoid edge noise
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    depth_roi = depth_img[max(0, cy-5):min(480, cy+5), max(0, cx-5):min(640, cx+5)]
                    
                    # Filter out 0 (invalid depth)
                    valid_depths = depth_roi[depth_roi > 0]
                    if len(valid_depths) > 0:
                        dist_m = np.median(valid_depths) / 1000.0
                    else:
                        dist_m = 0.0 # Unknown
                    
                    detections.append(f"{name}:{dist_m:.2f}m")
            
            if detections:
                msg = String()
                msg.data = ", ".join(detections)
                self.detection_pub.publish(msg)
                self.get_logger().info(f"👀 Found: {msg.data}")
                
        except Exception as e:
            self.get_logger().error(f"YOLO Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = Yolo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
