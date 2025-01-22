 
from launch import LaunchDescription 
from launch.actions import ExecuteProcess 
from ament_index_python.packages import get_package_share_directory 
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription 
from launch_ros.substitutions import FindPackageShare 
 
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription 
from launch.conditions import IfCondition, UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

    correlation_coefficient_arg = DeclareLaunchArgument(
            "correlation_coefficient", default_value = "0.99");


    relative_covariance_node = Node(
            package="relative_covariance",
            executable="relative_covariance_node",
            output="log",
            parameters = [
                {"correlation_coefficient": LaunchConfiguration("correlation_coefficient")},
                ],
            remappings = [
                ("pose_with_covariance", "/visual_slam/tracking/vo_pose_covariance"),
                ("pose_with_relative_covariance", "/visual_slam/tracking/vo_pose_with_relative_covariance")
                ]
            )

    return LaunchDescription([
        correlation_coefficient_arg,
        relative_covariance_node
        ])
