#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_test')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publishing to /cmd_vel...')

    def timer_callback(self):
        msg = Twist()
        # Move forward slowly
        msg.linear.x = 0.1
        # Rotate slowly
        msg.angular.z = 0.2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
