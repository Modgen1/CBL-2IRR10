import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class NavigationClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, '/tb1/navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = 4
        goal_msg.pose.pose.position.y = 4
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    nav_client.send_goal()
    rclpy.spin(nav_client)

if __name__ == '__main__':
    main()