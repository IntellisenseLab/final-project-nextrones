import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from semantic_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            DetectionArray, '/detections', self.detection_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10
        )
        
        # Publishers
        # We will publish Detection objects (label + 3D position) to /object_locations
        self.location_pub = self.create_publisher(Detection, '/object_locations', 10)
        
        self.bridge = CvBridge()
        self.latest_depth = None
        self.intrinsics = None 
        
        self.get_logger().info('✅ Object Localization Node Started (with TF support)')

    def info_callback(self, msg):
        try:
            self.intrinsics = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]
        except Exception as e:
            self.get_logger().error(f'❌ Localization Error: CameraInfo processing failed: {e}')

    def depth_callback(self, msg):
        try:
            # Kinect depth is 16UC1 (mm)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {e}')

    def detection_callback(self, msg):
        try:
            if self.latest_depth is None or self.intrinsics is None:
                return

            fx, fy, cx, cy = self.intrinsics
            
            for detection in msg.detections:
                try:
                    u, v = int(detection.position.x), int(detection.position.y)
                    
                    if u < 0 or u >= self.latest_depth.shape[1] or v < 0 or v >= self.latest_depth.shape[0]:
                        continue
                        
                    # Depth sampling with 3x3 window for robustness
                    u_min = max(0, u - 1)
                    u_max = min(self.latest_depth.shape[1], u + 2)
                    v_min = max(0, v - 1)
                    v_max = min(self.latest_depth.shape[0], v + 2)
                    
                    depth_roi = self.latest_depth[v_min:v_max, u_min:u_max]
                    valid_depths = depth_roi[depth_roi > 0]
                    
                    if len(valid_depths) == 0:
                        continue
                        
                    z_mm = np.median(valid_depths)
                    if z_mm > 5000: # Max range
                        continue
                        
                    z = float(z_mm) / 1000.0
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # Create a PointStamped in the camera's optical frame
                    p_cam = PointStamped()
                    p_cam.header = msg.header
                    p_cam.point.x = x
                    p_cam.point.y = y
                    p_cam.point.z = z
                    
                    # Transform to 'map' frame
                    try:
                        p_map = self.tf_buffer.transform(p_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                        
                        # Create final Detection message for the semantic map
                        out_msg = Detection()
                        out_msg.label = detection.label
                        out_msg.confidence = detection.confidence
                        out_msg.position = p_map.point # 3D position in map frame
                        
                        self.get_logger().info(f'✅ [Localization] {detection.label} at ({p_map.point.x:.2f}, {p_map.point.y:.2f})')
                        self.location_pub.publish(out_msg)
                        
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warning(f'⚠️ [Localization] TF Transform failed for {detection.label}: {e}')
                except Exception as e:
                    self.get_logger().error(f'❌ [Localization] Error processing detection {detection.label}: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ [Localization] Critical callback error: {e}')

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObjectLocalizationNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as e:
        print(f"❌ Critical Startup Error in Localization: {e}")

if __name__ == '__main__':
    main()

