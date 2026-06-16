import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
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
        self.bridge = CvBridge()
        self.get_logger().info('YOLO Detection Node Started')

    def compressed_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._detect_and_publish(cv_image, msg.header)

    def image_callback(self, msg):
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
                # Pixel coordinates of the center
                x_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                y_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                detection.u = x_center
                detection.v = y_center
                detection_array.detections.append(detection)
        
        self.publisher.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
