import rclpy
from rclpy.node import Node
from semantic_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo
import sys
import time
import numpy as np

class FullSystemVerifier(Node):
    def __init__(self):
        super().__init__('full_system_verifier')
        
        # Publishers to inject data at different stages
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.detection_pub = self.create_publisher(DetectionArray, '/detections', 10)
        
        # Subscribers to verify outputs
        self.loc_sub = self.create_subscription(Detection, '/object_locations', self.loc_cb, 10)
        self.marker_sub = self.create_subscription(MarkerArray, '/semantic_markers', self.marker_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        
        # Tracking states
        self.loc_received = False
        self.marker_received = False
        self.goal_received = False
        
        self.test_label = 'bottle'
        self.target_pos = (1.0, 2.0, 1.5) # x, y, z in camera frame

    def loc_cb(self, msg):
        self.get_logger().info(f'✅ [Localization] Received 3D location for {msg.label}')
        self.loc_received = True

    def marker_cb(self, msg):
        if len(msg.markers) > 0:
            self.get_logger().info(f'✅ [Semantic Map] Received {len(msg.markers)} markers')
            self.marker_received = True

    def goal_cb(self, msg):
        self.get_logger().info(f'✅ [Nav Goal] Received navigation goal for target {self.test_label}')
        self.goal_received = True

    def run_test(self):
        self.get_logger().info('🚀 Starting Comprehensive System Check...')
        time.sleep(2)
        
        # Inject Camera Info
        info = CameraInfo()
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0] # Simple intrinsics
        self.info_pub.publish(info)
        
        # Inject Detection
        det_arr = DetectionArray()
        det_arr.header.frame_id = 'camera_color_optical_frame'
        det_arr.header.stamp = self.get_clock().now().to_msg()
        d = Detection()
        d.label = self.test_label
        d.confidence = 0.9
        d.position.x = 320.0 # Center
        d.position.y = 240.0
        det_arr.detections.append(d)
        
        # Inject Depth (320x240 image with 1500mm depth)
        depth_img = Image()
        depth_img.header = det_arr.header
        depth_img.height = 480
        depth_img.width = 640
        depth_img.encoding = '16UC1'
        depth_data = np.full((480, 640), 1500, dtype=np.uint16)
        depth_img.data = depth_data.tobytes()
        depth_img.step = 640 * 2
        
        # Publish
        self.depth_pub.publish(depth_img)
        self.detection_pub.publish(det_arr)
        
        # Wait for results
        start_time = time.time()
        timeout = 10.0
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.loc_received and self.marker_received and self.goal_received:
                self.get_logger().info('🎉 FULL PIPELINE VERIFIED SUCCESSFULLY!')
                return True
        
        self.get_logger().error('❌ TIMEOUT: Some pipeline stages failed.')
        self.get_logger().info(f'Status - Loc: {self.loc_received}, Marker: {self.marker_received}, Goal: {self.goal_received}')
        return False

def main():
    rclpy.init()
    node = FullSystemVerifier()
    success = node.run_test()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
