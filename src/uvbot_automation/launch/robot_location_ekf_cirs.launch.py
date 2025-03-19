
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os.path

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="YellowBot",
            description="Top-level namespace",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")

    pkg_name = 'uvbot_automation'

    ekf_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'odometry_zyj.yaml')

    ekf_node = Node(
        package='robot_localization',
        namespace=namespace,
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    nodes = [
        ekf_node
    ]

    return LaunchDescription(declared_arguments + nodes)
