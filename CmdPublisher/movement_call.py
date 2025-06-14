import rclpy
from nav_goal_client import NavToGoalClient
from CoordinateCommunicator import CoordinateCommunicator

def goal_done_cb(robot, result):
    print(f"[{robot}] Navigation completed.")


def main():
    rclpy.init()

    tb1 = NavToGoalClient('tb1')
    tb2 = NavToGoalClient('tb2')
    tb3 = NavToGoalClient('tb3')
    tb4 = NavToGoalClient('tb4')

    tb1.send_goal(-3.2, 6.2, goal_done_cb)
    tb2.send_goal(1.5, -4.0, goal_done_cb)
    tb3.send_goal(2, 4, goal_done_cb)


    rclpy.spin(tb1)

if __name__ == '__main__':
    main()