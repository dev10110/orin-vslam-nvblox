from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diagnostics_monitor',
            executable='frequency_monitor',
            parameters = [
                { 'topic_name': 'chatter'},
                { 'topic_type': 'std_msgs/msg/String'},
                { 'min_frequency': 9.9},
                { 'max_frequency': 10.1}
                ],
            output="screen"
        ),
    ])
