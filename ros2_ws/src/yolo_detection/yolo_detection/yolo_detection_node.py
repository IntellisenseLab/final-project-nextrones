#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Load YOLOv8n model (Nano version for speed on RPi)
        self.get_logger().info('Loading YOLOv8n model...')
        self.model = YOLO('yolov8n.pt')
        
        self.last_inference_time = 0
        self.inference_interval = 1.0 / 5.0  # Limit to 5 FPS to save CPU
        
        self.get_logger().info('✅ YOLOv8 Detection Node Started')

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_inference_time < self.inference_interval:
            return
            
        self.last_inference_time = current_time
        
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference
        results = self.model(cv_image, verbose=False)
        
        # Process results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0]
                cls = box.cls[0]
                label = self.model.names[int(cls)]
                
                if conf > 0.5:
                    self.get_logger().info(f'Detected {label} with {conf:.2f} confidence')
                    
                    # (Optional) Draw on image for local viewing
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'{label} {conf:.2f}', (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # (Optional) Show locally if a monitor is connected
        # cv2.imshow("YOLOv8 Detection", cv_image)
        # cv2.waitKey(1)

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
