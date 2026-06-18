#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from semantic_msgs.msg import Detection
from geometry_msgs.msg import PoseStamped
import tf2_ros
import time

class NavGoalSenderNode(Node):
    def __init__(self):
        super().__init__('nav_goal_sender_node')

        # Parameters
        self.declare_parameter('target_object', 'bottle')
        self.target_object = self.get_parameter('target_object').get_parameter_value().string_value
        # How far short of the object to stop. Floor is ~robot_radius (0.175m) +
        # xy_goal_tolerance - any closer and the goal can land inside the obstacle's
        # inflated/lethal costmap cells (unreachable, causes planning failures), or
        # the goal_checker's tolerance could let the robot bump the object. Paired
        # with xy_goal_tolerance: 0.13 in navigation_params.yaml so the worst case
        # (0.3 - 0.13 = 0.17m) still just clears robot_radius.
        self.declare_parameter('standoff_distance', 0.3)
        self.standoff_distance = self.get_parameter('standoff_distance').get_parameter_value().double_value

        # TF2 to know where the robot currently is in the map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        # Listens to 3D localized object positions in the 'map' frame
        self.subscription = self.create_subscription(
            Detection,
            '/object_locations',
            self.object_callback,
            10
        )

        # Publishers
        # Sends goal to Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # State
        self.last_goal_sent_time = 0
        self.min_resend_interval = 2.0  # seconds; just a debounce, not a "wait for completion" gate
        self.last_goal_xy = None
        self.last_goal_yaw = None
        # Only send a fresh goal if it actually moved meaningfully. Without this,
        # frame-to-frame depth/detection jitter on the object position made every
        # ~10s resend publish a slightly different goal orientation, so the robot
        # kept restarting its RotateToGoal alignment and never settled - it just
        # looked like continuous rotation that never reached the goal.
        self.position_change_threshold = 0.15  # meters
        self.yaw_change_threshold = 0.2  # radians (~11 degrees)

        self.get_logger().info(f'✅ Nav Goal Sender Node Started. Current Target: {self.target_object}')

    def object_callback(self, msg):
        label = msg.label

        if label.lower() == self.target_object.lower():
            current_time = time.time()

            if current_time - self.last_goal_sent_time < self.min_resend_interval:
                return

            try:
                robot_tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warning(f'Could not get robot pose in map frame: {e}')
                return

            rx = robot_tf.transform.translation.x
            ry = robot_tf.transform.translation.y

            dx = msg.position.x - rx
            dy = msg.position.y - ry
            dist = math.hypot(dx, dy)

            if dist <= self.standoff_distance:
                self.get_logger().info(f'📍 {label} already within standoff distance ({dist:.2f}m), skipping goal.')
                return

            # Stop short of the object instead of driving onto it (the object
            # itself is an obstacle in the costmap, so a goal placed exactly
            # on it is unreachable and forces Nav2 into a Spin recovery loop)
            ratio = (dist - self.standoff_distance) / dist
            goal_x = rx + dx * ratio
            goal_y = ry + dy * ratio
            yaw = math.atan2(dy, dx)

            if self.last_goal_xy is not None:
                moved = math.hypot(goal_x - self.last_goal_xy[0], goal_y - self.last_goal_xy[1])
                yaw_diff = abs(math.atan2(math.sin(yaw - self.last_goal_yaw), math.cos(yaw - self.last_goal_yaw)))
                if moved < self.position_change_threshold and yaw_diff < self.yaw_change_threshold:
                    return

            self.get_logger().info(f'🎯 Target found: {label}. Sending standoff goal to Nav2!')

            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.position.z = 0.0

            # Face the object so the final orientation matches the approach heading
            goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

            self.goal_pub.publish(goal_msg)
            self.last_goal_sent_time = current_time
            self.last_goal_xy = (goal_x, goal_y)
            self.last_goal_yaw = yaw

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
