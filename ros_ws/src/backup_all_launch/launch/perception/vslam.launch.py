# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node, SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import math


def generate_launch_description():

    output_odom_frame_name_arg = DeclareLaunchArgument(
        'output_odom_frame_name', default_value='odom',
        description='The name of the VSLAM output frame')

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='vslam_container')
    enable_imu_fusion_arg = LaunchConfiguration('enable_imu_fusion', default=True)

    # define the bringup dir
    bringup_dir = get_package_share_directory('all_launch')

    # If we do not attach to a shared component container we have to create our own container.
    vslam_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # Vslam node
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode'),
            ])


    group_action = GroupAction([
        ##########################################
        ######### VISUAL SLAM NODE SETUP #########
        ##########################################
    
        # static transform between base_link and camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
                '--x', '0.040',
                '--y', '0.0',
                '--z', '0.040',
                '--roll', f"{math.pi}",
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
                ]
            ),

        # # Set general parameters
        # SetParameter(name='enable_debug_mode', value=False),
        # SetParameter(name='debug_dump_path', value='/tmp/cuvslam'),
        # SetParameter(name='enable_slam_visualization', value=True),
        # SetParameter(name='enable_observations_view', value=True),
        # SetParameter(name='enable_landmarks_view', value=True),
        # SetParameter(name='map_frame', value='map'),
        # SetParameter(name='enable_localization_n_mapping', value=False),
        # SetParameter(name='publish_odom_to_base_tf', value=True),
        # SetParameter(name='publish_map_to_odom_tf', value=False),
        # SetParameter(name='invert_odom_to_base_tf', value=True),

        # SetParameter(name='odom_frame', value=LaunchConfiguration('output_odom_frame_name')),

        # # Parameters for Realsense
        # SetParameter(name='enable_rectified_pose', value=True),
        # SetParameter(name='denoise_input_images', value=False),
        # SetParameter(name='rectified_images', value=True),
        # SetParameter(name='base_frame', value='base_link'),


        # # Parameters for Realsense + IMU
        # SetParameter(name="input_imu_frame", value="camera_gyro_optical_frame"),
        # SetParameter(name="input_left_camera_frame", value="camera_link"), # STOOOPID (but works)
        # SetParameter(name="enable_imu_fusion", value=False),
        # SetParameter(name="gyro_noise_density", value=0.000244),
        # SetParameter(name="gyro_random_walk", value=0.000019393),
        # SetParameter(name="accel_noise_density", value=0.001862),
        # SetParameter(name="accel_random_walk", value=0.003),
        # SetParameter(name="calibration_frequency", value=200.0),
        # SetParameter(name="img_jitter_threshold_ms", value=35.00),


        # Remappings for Realsense
        SetRemap(src=['/visual_slam/camera_info_0'],
                 dst=['/camera/infra1/camera_info']),
        SetRemap(src=['/visual_slam/camera_info_1'],
                 dst=['/camera/infra2/camera_info']),
        SetRemap(src=['/visual_slam/image_0'],
                 dst=['/camera/infra_1/image_rect_raw']),
        SetRemap(src=['/visual_slam/image_1'],
                 dst=['/camera/infra_2/image_rect_raw']),


        SetParameter(name='num_cameras', value= 2), 
        SetParameter(name='min_num_images', value=2), 
        SetParameter(name='enable_localization_n_mapping', value= False), 
        SetParameter(name='enable_imu_fusion', value= False), 
        SetParameter(name='gyro_noise_density', value= 0.000244), 
        SetParameter(name='gyro_random_walk', value= 0.000019393), 
        SetParameter(name='accel_noise_density', value= 0.001862), 
        SetParameter(name='accel_random_walk', value= 0.003), 
        SetParameter(name='calibration_frequency', value= 200.0), 
        SetParameter(name='rig_frame', value= 'base_link'), 
        SetParameter(name='imu_frame', value= 'front_stereo_camera_imu'), 
        SetParameter(name='enable_slam_visualization', value= True), 
        SetParameter(name='enable_landmarks_view', value= True), 
        SetParameter(name='enable_observations_view', value= True), 
        SetParameter(name='path_max_size', value= 200), 
        SetParameter(name='verbosity', value= 5), 
        SetParameter(name='enable_debug_mode', value= False), 
        SetParameter(name='debug_dump_path', value= '/tmp/cuvslam'), 
        SetParameter(name='map_frame', value= 'map'), 
        SetParameter(name='odom_frame', value= 'odom'), 
        SetParameter(name='base_frame', value= 'base_link'), 
        SetParameter(name='enable_rectified_pose', value=True),
        SetParameter(name='enable_image_denoising', value=False),
        SetParameter(name='rectified_images', value=True),
        SetParameter(name='camera_optical_frames', value=[
            'camera_infra1_optical_frame',
            'camera_infra2_optical_frame',
        ]),


        # ADD COMPOSABLE NODES
        load_composable_nodes
    ])

    return LaunchDescription([output_odom_frame_name_arg,
                              vslam_container,
                              group_action])
