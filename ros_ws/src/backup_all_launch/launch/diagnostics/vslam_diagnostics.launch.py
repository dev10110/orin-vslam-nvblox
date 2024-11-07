import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import yaml

def load_topics(config_file):
    topics = []
    with open(config_file, 'r') as fp:
      f = yaml.safe_load(fp)
      for d in f["topics"]:
          t = list(d.values())[0]
          topics.append(t)

    return topics

def frequency_monitor_node(topic):

    node = Node(
            package="diagnostics_monitor",
            executable="frequency_monitor",
            parameters = [
                { 'topic_name': topic["topic_name"]},
                { 'topic_type': topic["topic_type"]},
                { 'min_frequency': topic["min_frequency"]},
                { 'max_frequency': topic["max_frequency"]}
                ],
            output="screen"
            )

    return node


def generate_launch_description():

    bringup_dir = get_package_share_directory('all_launch')


    # define actions
    actions = []

    # declare params
    run_gui_arg = DeclareLaunchArgument(
            "run_gui", default_value="True")


    actions = actions + [run_gui_arg]


    # create the frequency monitor nodes
    topics_config_path = os.path.join(
            bringup_dir, 'config', 'diagnostics', 'vslam_topics.yaml')
    topics = load_topics(topics_config_path)
    frequency_monitor_nodes = [frequency_monitor_node(t) for t in topics]

    actions = actions + frequency_monitor_nodes

    # create the vslam monitor node
    vslam_monitor_node = Node(
            package="diagnostics_monitor",
            executable="vslam_monitor",
            output="screen"
            )

    actions = actions + [vslam_monitor_node]

    # create the aggregator
    analyzers_filepath = os.path.join(
            bringup_dir, "config", "diagnostics", "vslam_analyzers.yaml")
    aggregator = Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            output="screen",
            parameters=[analyzers_filepath])

    actions = actions + [aggregator]

    return LaunchDescription(actions)
