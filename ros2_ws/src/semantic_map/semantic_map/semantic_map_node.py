import rclpy
from rclpy.node import Node
from semantic_msgs.msg import Detection
from visualization_msgs.msg import Marker, MarkerArray
import math

class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')
        
        # Subscription to localized 3D detections
        self.subscription = self.create_subscription(
            Detection, '/object_locations', self.object_callback, 10
        )
        
        # Publisher for RViz2 markers
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_markers', 10)
        
        # Object store: list of dictionaries {label, x, y, z}
        self.object_store = []
        self.deduplication_threshold = 0.5 # 0.5 meters as per proposal
        
        self.get_logger().info('✅ Semantic Map Node Started')

    def object_callback(self, msg):
        new_pos = msg.position
        label = msg.label
        
        # Deduplication check
        is_duplicate = False
        for obj in self.object_store:
            if obj['label'] == label:
                # Calculate Euclidean distance
                dist = math.sqrt(
                    (new_pos.x - obj['x'])**2 + 
                    (new_pos.y - obj['y'])**2 + 
                    (new_pos.z - obj['z'])**2
                )
                if dist < self.deduplication_threshold:
                    is_duplicate = True
                    # Optionally update position to be more accurate (averaging)
                    obj['x'] = (obj['x'] + new_pos.x) / 2.0
                    obj['y'] = (obj['y'] + new_pos.y) / 2.0
                    obj['z'] = (obj['z'] + new_pos.z) / 2.0
                    break
        
        if not is_duplicate:
            self.get_logger().info(f'New {label} added to semantic map at ({new_pos.x:.2f}, {new_pos.y:.2f})')
            self.object_store.append({
                'label': label,
                'x': new_pos.x,
                'y': new_pos.y,
                'z': new_pos.z
            })
        
        # Always publish the latest markers
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.object_store):
            # Create a sphere marker for the object
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "semantic_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obj['x']
            marker.pose.position.y = obj['y']
            marker.pose.position.z = obj['z']
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0 # Opacity
            
            # Simple color assignment based on label
            if obj['label'] == 'person':
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            elif obj['label'] == 'bottle':
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
            else:
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0
                
            marker_array.markers.append(marker)
            
            # Add a text label marker
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "semantic_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj['x']
            text_marker.pose.position.y = obj['y']
            text_marker.pose.position.z = obj['z'] + 0.3 # Slightly above the sphere
            text_marker.scale.z = 0.15 # Text size
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0
            text_marker.text = obj['label']
            
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
