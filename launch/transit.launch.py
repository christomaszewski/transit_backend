"""Launch file for TRANSIT node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("transit_backend"), "config")
    default_params = os.path.join(config_dir, "default_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to parameters YAML file",
        ),
        Node(
            package="transit_backend",
            executable="transit_node",
            name="transit_node",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
            emulate_tty=True,
        ),
    ])
