import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_message)
        #initializing timer
        self.start_time = self.get_clock().now()

    #msg to publish to cmd_vel
    def publish_message(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > 5.0:
            msg = Twist()
            self.publisher_.publish(msg)
            self.get_logger().info('Stopping publisher')
            rclpy.shutdown()
            return
        msg = Twist()
        msg.linear.x = 0.2
        #msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing velocity command')

def main(args=None):
    rclpy.init(args=args)
    node = VelPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
