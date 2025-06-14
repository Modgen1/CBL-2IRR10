import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

def launch_setup(context, *args, **kwargs):
    # Get the share directory for various packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox') # For SLAM if needed

    # --- Launch Arguments ---
    # Gazebo World
    world_file_name = LaunchConfiguration('world_file_name', default='turtlebot3_world.world') # <--- INPUT: Your Gazebo world file name
    world_path = os.path.join(pkg_turtlebot3_gazebo, 'worlds', world_file_name.perform(context)) # <--- INPUT: Path to your Gazebo world

    # Robot Count
    num_robots = LaunchConfiguration('num_robots', default='3') # <--- INPUT: Number of robots to spawn

    # Initial X/Y/Yaw for robots (example: adjust as needed)
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')

    # Use SLAM or Nav2 Localization
    use_slam = LaunchConfiguration('use_slam', default='False') # Set to True for SLAM, False for AMCL (map_server)
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_nav2_bringup, 'maps', 'turtlebot3_world.yaml')) # <--- INPUT: Path to your pre-built map for AMCL

    # Use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # RViz configuration
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    rviz_config_file = LaunchConfiguration('rviz_config_file',
                                           default=os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')) # <--- INPUT: Your RViz config file

    # --- Gazebo Launch ---
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # --- Robot Spawning and Nav2 Setup Loop ---
    robot_spawns_and_nav2 = []
    num_robots_val = int(num_robots.perform(context))

    for i in range(num_robots_val):
        robot_name = f'tb3_{i}' # <--- INPUT: Naming convention for your robots
        robot_namespace = f'/{robot_name}'

        # Unique initial positions (example: adjust for your world)
        # You'll likely want to define these more carefully based on your world map
        # For simplicity, here's a basic offset
        spawn_x_offset = float(initial_x.perform(context)) + (i * 0.5)
        spawn_y_offset = float(initial_y.perform(context)) + (i * 0.5)
        spawn_yaw_offset = float(initial_yaw.perform(context))

        # URDF file for TurtleBot3 Burger
        urdf_file = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')

        # <--- INPUT: Custom Nav2 parameters for each robot.
        # You might want to make these unique per robot.
        # E.g., `os.path.join(get_package_share_directory('YOUR_ROBOT_CUSTOM_CONFIG_PKG'), 'params', f'nav2_params_{robot_name}.yaml')`
        # For this example, we'll use a generic one and rely on namespace remapping for topics.
        nav2_params_file = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml') # Default Nav2 parameters
        # If you have robot-specific nav2_params files, specify them here:
        # nav2_params_file = os.path.join(get_package_share_directory('my_robot_config_pkg'), 'params', f'nav2_params_{robot_name}.yaml')

        # Push ROS namespace for each robot
        robot_group = GroupAction([
            PushRosNamespace(robot_namespace),

            # Spawn robot in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-entity', robot_name,
                    '-file', urdf_file,
                    '-x', str(spawn_x_offset),
                    '-y', str(spawn_y_offset),
                    '-Y', str(spawn_yaw_offset),
                    '-robot_namespace', robot_namespace # Essential for namespacing in Gazebo model states
                ]
            ),

            # Robot State Publisher (for TF)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': TextSubstitution(text=open(urdf_file, 'r').read())
                }]
            ),

            # --- Nav2 Bringup for each robot ---
            # Define specific Nav2 parameters for each robot, if different from default
            # Otherwise, these will apply after the namespace push
            # You will likely have custom config files for each robot!
            # Example: nav2_params_tb3_0.yaml, nav2_params_tb3_1.yaml etc.
            # Make sure these are in a 'params' directory within your custom package share.
            # For simplicity, using a generic one here.

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_slam': use_slam,
                    'namespace': robot_namespace # This ensures Nav2 nodes launch in the correct namespace
                }.items()
            ),
        ])
        robot_spawns_and_nav2.append(robot_group)

    # --- RViz Launch (optional) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        DeclareLaunchArgument(
            'world_file_name',
            default_value='turtlebot3_world.world',
            description='Gazebo world file name'),
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='Number of TurtleBot3 robots to spawn'),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Initial X position for the first robot'),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial Y position for the first robot'),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Initial Yaw position for the first robot'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='False',
            description='Whether to use SLAM (True) or AMCL (False)'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav2_bringup, 'maps', 'turtlebot3_world.yaml'),
            description='Path to the map YAML file (if not using SLAM)'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to launch RViz'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'),
            description='Path to the RViz config file'),
        
        gazebo_launch,
        *robot_spawns_and_nav2, # Unpack the list of robot groups
        rviz_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
