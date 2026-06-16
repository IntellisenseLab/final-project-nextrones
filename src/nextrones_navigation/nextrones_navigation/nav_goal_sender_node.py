import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nextrones_interfaces.msg import LocalizedObject, LocalizedObjectArray

class NavGoalSenderNode(Node):
    def __init__(self):
        super().__init__('nav_goal_sender_node')
        
        self.declare_parameter('target_label', 'bottle')
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(LocalizedObjectArray, '/nextrones/localized_objects', self.objects_callback, 10)
        
        # AI Sleep Switch publisher and timer
        self.ai_active_pub = self.create_publisher(Bool, '/nextrones/ai_active', 10)
        self.ai_active = True
        self.create_timer(1.0, self.publish_ai_state)
        
        self.goal_sent = False
        self.get_logger().info(f'Nav Goal Sender Node Started. Looking for: {self.target_label}')

    def publish_ai_state(self):
        msg = Bool()
        msg.data = self.ai_active
        self.ai_active_pub.publish(msg)

    def objects_callback(self, msg):
        if self.goal_sent:
            return
            
        for obj in msg.objects:
            if obj.label == self.target_label:
                self.send_goal(obj.point)
                self.goal_sent = True
                self.ai_active = False  # Turn off YOLO to save CPU
                self.get_logger().info('Target found! Sent sleep signal to YOLO/OpenCV.')
                break

    def send_goal(self, point_stamped):
        self.get_logger().info(f'Found {self.target_label}! Sending goal to Nav2...')
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header = point_stamped.header
        goal_msg.pose.pose.position = point_stamped.point
        # Standard orientation (facing forward)
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
