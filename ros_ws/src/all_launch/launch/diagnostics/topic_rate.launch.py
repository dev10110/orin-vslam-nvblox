import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # config file
    config_file = os.path.join(
        get_package_share_directory('all_launch'),
        'config', 'diagnostics', 'topic_rates.yaml')


    # vicon bridge
    monitor_node = Node(
        package="topic_rate_monitor",
        executable="topic_rate_monitor",
        parameters=[
            {"topics": config_file}
            ],
        )

    return LaunchDescription([
        monitor_node,
        ])
