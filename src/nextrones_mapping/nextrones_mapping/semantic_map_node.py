import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nextrones_interfaces.msg import LocalizedObject, LocalizedObjectArray
import math

class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')
        
        self.objects = [] # List of {'label': string, 'x': float, 'y': float, 'z': float}
        self.dist_threshold = 0.5
        
        self.create_subscription(LocalizedObjectArray, '/nextrones/localized_objects', self.objects_callback, 10)
        self.publisher = self.create_publisher(MarkerArray, '/nextrones/semantic_markers', 10)
        
        self.get_logger().info('Semantic Map Node Started')

    def objects_callback(self, msg):
        changed = False
        for loc_obj in msg.objects:
            if self.is_new_object(loc_obj):
                self.objects.append({
                    'label': loc_obj.label,
                    'x': loc_obj.point.point.x,
                    'y': loc_obj.point.point.y,
                    'z': loc_obj.point.point.z
                })
                self.get_logger().info(f'Added new {loc_obj.label} to semantic map at ({loc_obj.point.point.x:.2f}, {loc_obj.point.point.y:.2f})')
                changed = True
        
        if changed:
            self.publish_markers()

    def is_new_object(self, loc_obj):
        for obj in self.objects:
            if obj['label'] == loc_obj.label:
                dist = math.sqrt(
                    (obj['x'] - loc_obj.point.point.x)**2 +
                    (obj['y'] - loc_obj.point.point.y)**2
                )
                if dist < self.dist_threshold:
                    return False
        return True

    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.objects):
            # Sphere Marker
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'objects'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = obj['x']
            sphere.pose.position.y = obj['y']
            sphere.pose.position.z = obj['z']
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            marker_array.markers.append(sphere)
            
            # Text Marker
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'labels'
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = obj['x']
            text.pose.position.y = obj['y']
            text.pose.position.z = obj['z'] + 0.3
            text.scale.z = 0.15
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = obj['label']
            marker_array.markers.append(text)
            
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
