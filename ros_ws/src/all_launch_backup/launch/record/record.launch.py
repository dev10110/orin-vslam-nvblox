import launch

def generate_launch_description():

    bagname = "run2"

    topics = [
       # "/camera/infra1/camera_info",
       # "/camera/infra1/image_rect_raw",
       # "/camera/infra1/metadata",
       # "/diagnostics",
       # "/diagnostics_agg",
       "/joy",
       # "/parameter_events",
       "/px4_2/fmu/in/commander_set_state",
       "/px4_2/fmu/in/trajectory_setpoint",
       "/px4_2/fmu/in/vehicle_visual_odometry",
       "/px4_2/fmu/out/commander_status",
       "/px4_2/fmu/out/simple_battery_status",
       "/px4_2/fmu/out/timesync_status",
       # "/px4_2/fmu/out/vehicle_attitude",
       "/px4_2/fmu/out/vehicle_local_position",
       "/px4_2/fmu/out/vehicle_status",
       # "/px4_2/viz/trajectory_setpoint",
       "/tf",
       "/tf_static",
       # "/vicon/px4_2/px4_2",
       "/visual_slam/status",
       # "/visual_slam/tracking/vo_path",
       "/visual_slam/tracking/vo_pose",
       "/visual_slam/tracking/vo_pose_covariance",
       # "/visual_slam/vis/observations_cloud"
       ]

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics, #  + ["--qos-profile-overrides-path", "/root/ros_ws/src/all_launch/config/record/qos_overrides.yaml"], #  + ["-o", bagname],
            cwd=["/root/ros_ws/bags"],
            output='screen'
        )
    ])
