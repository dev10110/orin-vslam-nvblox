
from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu
import math

import yaml

NVBLOX_CONTAINER_NAME = 'nvblox_container'

def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    # record node
    record_node = ComposableNode(
        namespace = "camera",
        package = "rosbag2_transport", 
        plugin = "rosbag2_transport::Recorder",
        name = "record",
        parameters = dump_params("/root/ros_ws/src/all_launch/config/ros2bag/recorder.yaml", "recorder_params_node"),
        extra_arguments = [
            {"use_intra_process_comms": False}
        ]
    )

    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            [record_node],
        ))


    return LaunchDescription(actions)
