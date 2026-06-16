import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from nextrones_interfaces.msg import Detection, DetectionArray
from ultralytics import YOLO
import cv2

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        # When True, subscribe to a CompressedImage topic instead of raw Image.
        # Used when YOLO runs off-board (PC) and the camera streams over WiFi.
        self.declare_parameter('use_compressed', False)

        model_path = self.get_parameter('model').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value

        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)

        if use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage, image_topic, self.compressed_callback, 10)
            self.get_logger().info(f'Subscribing to COMPRESSED images: {image_topic}')
        else:
            self.subscription = self.create_subscription(
                Image, image_topic, self.image_callback, 10)

        self.publisher = self.create_publisher(DetectionArray, '/nextrones/detections', 10)
        self.debug_publisher = self.create_publisher(Image, '/nextrones/debug_image', 10)
        self.bridge = CvBridge()
        
        self.ai_active = True
        self.frame_counter = 0
        self.create_subscription(Bool, '/nextrones/ai_active', self.ai_state_callback, 10)
        
        self.get_logger().info('YOLO Detection Node Started')

    def ai_state_callback(self, msg):
        if self.ai_active and not msg.data:
            self.get_logger().info("AI SLEEP SIGNAL RECEIVED. Pausing heavy cv2/YOLO processing to save CPU.")
        self.ai_active = msg.data

    def compressed_callback(self, msg):
        if not self.ai_active:
            return
            
        self.frame_counter += 1
        if self.frame_counter % 6 != 0:
            return
            
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._detect_and_publish(cv_image, msg.header)

    def image_callback(self, msg):
        if not self.ai_active:
            return
            
        self.frame_counter += 1
        if self.frame_counter % 6 != 0:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._detect_and_publish(cv_image, msg.header)

    def _detect_and_publish(self, cv_image, header):
        results = self.model(cv_image, verbose=False)
        
        detection_array = DetectionArray()
        detection_array.header = header
        
        for result in results:
            for box in result.boxes:
                detection = Detection()
                detection.label = self.model.names[int(box.cls[0])]
                detection.confidence = float(box.conf[0])
                # Get bounding box corners
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Pixel coordinates of the center
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)
                detection.u = x_center
                detection.v = y_center
                detection_array.detections.append(detection)
                
                # Draw debug info
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{detection.label} {detection.confidence:.2f}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(cv_image, (x_center, y_center), 5, (0, 0, 255), -1)
        
        self.publisher.publish(detection_array)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = header
        self.debug_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
