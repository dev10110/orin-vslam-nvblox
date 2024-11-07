import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_dir = get_package_share_directory('all_launch')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='False',
        description='Whether to start RVIZ')
    run_foxglove_arg = DeclareLaunchArgument(
        'run_foxglove', default_value="False")
    run_vslam_arg = DeclareLaunchArgument('run_vslam', default_value='True')
    run_nvblox_arg = DeclareLaunchArgument('run_nvblox', default_value='False')
    run_diagnostics_arg = DeclareLaunchArgument('run_diagnostics', default_value='True')

    from_bag_arg = DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live realsense data')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='rosbag2*',
        description='Path of the bag (only used if from_bag == True)')
    global_frame = LaunchConfiguration('global_frame',
                                       default='odom')

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')

    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name}.items(),
        condition=UnlessCondition(LaunchConfiguration('from_bag')))

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame,
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items(),
        condition=IfCondition(LaunchConfiguration('run_vslam'))
    )

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_realsense': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items(),
        condition=IfCondition(LaunchConfiguration('run_nvblox'))
    )

    # Ros2 bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        shell=True, output='screen',
        condition=IfCondition(LaunchConfiguration('from_bag')))

    # run diagnostics
    diagnostics_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource( [os.path.join(
                bringup_dir, "launch", "diagnostics", "vslam_diagnostics.launch.py")] ),
            launch_arguments={'run_gui': "True"}.items(),
            condition=IfCondition(LaunchConfiguration('run_diagnostics'))
            )


    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={'config_name': 'basic.rviz',
                          'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    # Foxglove
    foxglove_bridge_dir = get_package_share_directory("foxglove_bridge")
    foxglove_launch_file = os.path.join(foxglove_bridge_dir, 'launch', 'foxglove_bridge_launch.xml')
    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_file),
        condition=IfCondition(LaunchConfiguration('run_foxglove'))
        )

    return LaunchDescription([
        run_rviz_arg,
        run_foxglove_arg,
        run_vslam_arg,
        run_nvblox_arg,
        run_diagnostics_arg,
        from_bag_arg,
        bag_path_arg,
        shared_container,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        bag_play,
        diagnostics_launch,
        rviz_launch,
        foxglove_launch,
        ])
