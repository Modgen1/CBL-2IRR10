import os

from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='TB3_2')  # pass from outer launch or CLI
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    pkg_dir = get_package_share_directory('multi_cartographer')
    config_dir = os.path.join(pkg_dir, 'config')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='TB3_2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # This applies the namespace to everything after it
        PushRosNamespace(namespace),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'turtlebot3_lds_2d_r2.lua'
            ]
        ),

	Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'publish_period_sec': 1.0}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        )
    ])

