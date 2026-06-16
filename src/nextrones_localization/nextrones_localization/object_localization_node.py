import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from nextrones_interfaces.msg import Detection, DetectionArray, LocalizedObject, LocalizedObjectArray
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')

        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('max_depth', 4.5)
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_image = None
        self.camera_info = None

        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10)
        self.create_subscription(DetectionArray, '/nextrones/detections', self.detections_callback, 10)

        self.publisher = self.create_publisher(LocalizedObjectArray, '/nextrones/localized_objects', 10)
        self.get_logger().info(f'Object Localization Node Started (depth range: {self.min_depth}–{self.max_depth} m)')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def info_callback(self, msg):
        self.camera_info = msg

    def detections_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return
        
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        object_array = LocalizedObjectArray()
        
        for detection in msg.detections:
            u, v = detection.u, detection.v
            
            if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
                continue
                
            depth = self.depth_image[v, u]
            
            if np.isnan(depth) or depth <= 0:
                continue
            
            if self.depth_image.dtype == np.uint16:
                z = float(depth) / 1000.0
            else:
                z = float(depth)
            
            if z < self.min_depth or z > self.max_depth:
                continue
                
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            
            point_camera = PointStamped()
            point_camera.header = msg.header
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z
            
            try:
                # Use time=0 (latest available transform) to avoid clock-skew
                # between the camera image bridge and AMCL's TF publication.
                point_camera.header.stamp = rclpy.time.Time().to_msg()
                point_map = self.tf_buffer.transform(point_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                
                loc_obj = LocalizedObject()
                loc_obj.label = detection.label
                loc_obj.point = point_map
                object_array.objects.append(loc_obj)
                
            except Exception as e:
                self.get_logger().warn(f'Could not transform point: {e}')
        
        if len(object_array.objects) > 0:
            self.publisher.publish(object_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
