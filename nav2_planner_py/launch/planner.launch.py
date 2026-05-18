"""
Launch file for the Nav2 Planner Server.

This launch file starts the nav2_planner node which provides path planning
services for navigation using the ComputePathToPose and ComputePathThroughPoses
action servers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for nav2_planner."""

    # Get the package directory
    package_dir = FindPackageShare(package='nav2_planner_py')

    # Path to default configuration file
    default_config_path = PathJoinSubstitution([package_dir, 'params', 'default.yaml'])

    # Launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

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
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')

    # Planner node
    planner_node = Node(
        package='nav2_planner_py',
        executable='nav2_planner',
        name='planner',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Create launch description
    ld = LaunchDescription()

    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_log_level_arg)
    ld.add_action(declare_params_file_arg)

    # Use GroupAction to apply namespace
    group_action = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            planner_node,
        ]
    )

    ld.add_action(group_action)

    return ld
