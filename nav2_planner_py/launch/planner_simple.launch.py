"""
Minimal launch file for the Nav2 Planner Server.

This is a simple launch file that starts the nav2_planner node with
configuration from default.yaml file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate minimal launch description for nav2_planner."""

    # Get the package directory
    package_dir = FindPackageShare(package='nav2_planner_py')

    # Path to default configuration file
    default_config_path = PathJoinSubstitution([package_dir, 'params', 'default.yaml'])

    # Launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the planner node'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Full path to the parameters configuration file'
    )

    # Get substitution values
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')

    # Planner node
    planner_node = Node(
        package='nav2_planner_py',
        executable='nav2_planner',
        name='planner',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # Create launch description
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_log_level_arg)
    ld.add_action(declare_params_file_arg)
