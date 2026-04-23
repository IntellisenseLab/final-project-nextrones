#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time
import os
from semantic_msgs.msg import Detection, DetectionArray

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos)

        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('debug_view', False)
        self.declare_parameter('publish_debug', False)  # Disable WiFi bandwidth-heavy images
        self.declare_parameter('inference_fps', 2.0)  # Throttled for Pi 5 performance
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value
        self.publish_debug = self.get_parameter('publish_debug').get_parameter_value().bool_value
        self.inference_fps = self.get_parameter('inference_fps').get_parameter_value().double_value
        
        # Load YOLOv8n model
        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.model = YOLO(model_path)
        
        self.last_inference_time = 0
        self.inference_interval = 1.0 / self.inference_fps if self.inference_fps > 0 else 0.0

        self.get_logger().info(f'✅ YOLOv8 Detection Node Started (FPS: {self.inference_fps})')
        
        self.publisher = self.create_publisher(DetectionArray, '/detections', 10)
        self.debug_img_pub = self.create_publisher(CompressedImage, '/yolo_debug/image/compressed', qos)
        self.debug_small_pub = self.create_publisher(Image, '/yolo_debug/image/small', qos)


    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_inference_time < self.inference_interval:
            return
            
        self.last_inference_time = current_time
        
        # Log frame arrival
        if not hasattr(self, '_frame_count'): self._frame_count = 0
        self._frame_count += 1
        if self._frame_count % 50 == 0:
            self.get_logger().info(f'🖼️ YOLO: Frame sync OK ({self._frame_count} frames received)')
        
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        
        if cv_image is None:
            return
        
        # Run YOLO inference (Optimized for speed with imgsz=320)
        try:
            results = self.model(cv_image, verbose=False, stream=True, imgsz=320)
            
            detection_array = DetectionArray()
            detection_array.header = msg.header
            
            for r in results:
                for box in r.boxes:
                    # Filter by confidence
                    conf = float(box.conf[0])
                    if conf < 0.3:
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
            if self.debug_view or True: # Always publish to topic for RViz
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
                
            # 🔹 Publish optimized streams for PC Viewing (Disabled by default on Pi for bandwidth)
            if self.publish_debug:
                try:
                    # 1. Compressed Stream (Optimized for maximum FPS)
                    success, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 40])
                    if success:
                        compressed_msg = CompressedImage()
                        compressed_msg.header = msg.header
                        compressed_msg.format = "jpeg"
                        compressed_msg.data = np.array(buffer).tobytes()
                        self.debug_img_pub.publish(compressed_msg)
                    
                    # 2. Universal Small Stream (Works WITHOUT plugins on any PC)
                    small_frame = cv2.resize(cv_image, (320, 240))
                    small_msg = self.bridge.cv2_to_imgmsg(small_frame, encoding="bgr8")
                    small_msg.header = msg.header
                    self.debug_small_pub.publish(small_msg)

                except Exception as e:
                    self.get_logger().error(f"Failed to publish debug images: {e}")

                # Optional local display
                if self.debug_view and os.environ.get('DISPLAY'):
                    cv2.imshow('YOLOv8 Detection View', cv_image)
                    cv2.waitKey(1)



        except Exception as e:
            self.get_logger().error(f'❌ YOLO Inference Error: {e}')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = YoloDetectionNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as e:
        print(f"❌ Critical Startup Error: {e}")

if __name__ == '__main__':
    main()

