import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class publishSubscribe(Node):
    def __init__(self):
        super().__init__('publish_subscribe')

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to /odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer to publish velocity
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Flags and tracking
        self.initial_position = None
        self.stopped = False

    def odom_callback(self, msg):
        pos = msg.pose.pose.position

        if self.initial_position is None:
            self.initial_position = pos
            return

        # Calculate distance from start
        dx = pos.x - self.initial_position.x
        dy = pos.y - self.initial_position.y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f'Distance traveled: {distance:.2f} m')

        if distance >= 1.0 and not self.stopped:
            self.stop_robot()

    def publish_cmd(self):
        if self.stopped:
            return

        msg = Twist()
        msg.linear.x = 0.2  # move forward
        #msg.angular.z = 0.5
        self.cmd_pub.publish(msg)
        self.get_logger().info('Publishing forward velocity')

    def stop_robot(self):
        self.get_logger().info('Distance reached. Stopping robot.')
        stop_msg = Twist()  # all zeros
        self.cmd_pub.publish(stop_msg)
        self.stopped = True
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = publishSubscribe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()