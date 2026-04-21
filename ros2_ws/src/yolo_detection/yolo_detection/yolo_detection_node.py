#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time
from semantic_msgs.msg import Detection, DetectionArray

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_callback,
            qos)
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('model_path', '/home/hasini/yolov8n.pt')
        self.declare_parameter('debug_view', True)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value
        
        # Load YOLOv8n model
        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.model = YOLO(model_path)
        
        self.last_inference_time = 0
        self.inference_interval = 1.0 / 10.0  # Limit to 10 FPS
        
        self.get_logger().info('✅ YOLOv8 Detection Node Started')
        
        self.publisher = self.create_publisher(DetectionArray, '/detections', 10)

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_inference_time < self.inference_interval:
            return
            
        self.last_inference_time = current_time
        
        # Log frame arrival (Nature-style tracking)
        if not hasattr(self, '_frame_count'): self._frame_count = 0
        self._frame_count += 1
        if self._frame_count % 50 == 0:
            self.get_logger().info(f'🖼️ YOLO: Frame sync OK ({self._frame_count} frames received from Pi)')
        
        # Convert ROS CompressedImage to OpenCV format
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            return
        
        if cv_image is None:
            return
        
        # Run YOLO inference
        try:
            results = self.model(cv_image, verbose=False, stream=True)
            
            detection_array = DetectionArray()
            detection_array.header = msg.header
            
            for r in results:
                for box in r.boxes:
                    # Filter by confidence
                    conf = float(box.conf[0])
                    if conf < 0.5:
                        continue
                        
                    # Get coordinates
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    
                    # Calculate the center (u, v) and area
                    u = (x1 + x2) / 2.0
                    v = (y1 + y2) / 2.0
                    area = (x2 - x1) * (y2 - y1)
                    
                    label = self.model.names[int(box.cls[0])]
                    self.get_logger().info(f'Detected {label} | Center: ({u:.1f}, {v:.1f}) | Area: {area:.0f}')
                    
                    # Fill custom message
                    detection = Detection()
                    detection.label = label
                    detection.confidence = conf
                    # Store center (u, v) in position
                    detection.position.x = u
                    detection.position.y = v
                    detection.position.z = 0.0
                    
                    # Store width, height, and area in size
                    detection.size.x = float(x2 - x1)
                    detection.size.y = float(y2 - y1)
                    detection.size.z = float(area)
                    
                    detection_array.detections.append(detection)
            
            if detection_array.detections:
                self.publisher.publish(detection_array)
            
            # Visualization
            if self.debug_view:
                # Draw detections on image
                for det in detection_array.detections:
                    u, v = int(det.position.x), int(det.position.y)
                    w, h = int(det.size.x), int(det.size.y)
                    x1, y1 = int(u - w/2), int(v - h/2)
                    x2, y2 = int(u + w/2), int(v + h/2)
                    
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"{det.label} {det.confidence:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.imshow('YOLOv8 Detection View', cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'❌ YOLO Inference Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
