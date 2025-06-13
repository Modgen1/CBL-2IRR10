import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavToGoalClient(Node):
    def __init__(self, robot_name):
        super().__init__(f'nav_to_pose_client_{robot_name}')
        self.robot_name = robot_name
        self._action_client = ActionClient(
            self, NavigateToPose, f'/{self.robot_name}/navigate_to_pose'
        )

    def send_goal(self, x, y, done_cb=None):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal to {self.robot_name}: ({x}, {y})')

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_cb(future, done_cb)
        )

    def _goal_response_cb(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future: self._result_cb(result_future, done_cb)
        )

    def _result_cb(self, future, done_cb):
        result = future.result().result
        self.get_logger().info(f'{self.robot_name} reached the goal!')

        if done_cb:
            done_cb(self.robot_name, result)