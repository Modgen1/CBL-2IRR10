from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_cartographer')

    robot1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot.launch.py')
        ),
	launch_arguments={'namespace': 'TB3_1'}.items(),
    )

    robot2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot2.launch.py')
        ),
	launch_arguments={'namespace': 'TB3_2'}.items(),
    )

    return LaunchDescription([
        robot1_launch,
        robot2_launch,
    ])

