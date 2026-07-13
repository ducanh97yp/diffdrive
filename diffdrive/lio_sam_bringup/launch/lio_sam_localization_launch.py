import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    diff_description_share = get_package_share_directory('diff_description')
    lio_sam_bringup_share = get_package_share_directory('lio_sam_bringup')
    lio_sam_share = get_package_share_directory('lio_sam')

    world = LaunchConfiguration('world')
    rviz_use = LaunchConfiguration('rviz')
    map_pcd_path = LaunchConfiguration('map_pcd_path')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_yaw = LaunchConfiguration('initial_yaw')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='house.world',
        description='Gazebo world file (from diff_description/worlds)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch LIO-SAM RViz view'
    )
    declare_map_pcd_cmd = DeclareLaunchArgument(
        'map_pcd_path',
        description='Path to the GlobalMap.pcd saved earlier via the lio_sam/save_map service'
    )
    # Robot spawns at the same pose every run (see diff_description's world/spawn args),
    # but that pose is rarely where mapping was originally started from - these seed
    # the ICP search so it converges on the right place in the saved map instead of
    # drifting to the nearest self-similar room. Override per-run, or nudge with
    # RViz's "2D Pose Estimate" (published on /initialpose) once running.
    declare_initial_x_cmd = DeclareLaunchArgument('initial_x', default_value='0.0')
    declare_initial_y_cmd = DeclareLaunchArgument('initial_y', default_value='0.0')
    declare_initial_z_cmd = DeclareLaunchArgument('initial_z', default_value='0.0')
    declare_initial_yaw_cmd = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_description_share, 'launch', 'rviz_joint_control.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    params_file = PathJoinSubstitution([lio_sam_bringup_share, 'config', 'lio_sam_params.yaml'])

    adapter_node = Node(
        package='lio_sam_bringup',
        executable='gz_lidar_to_ouster',
        name='gz_lidar_to_ouster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Replaces lio_sam_slam_launch.py's static identity map->odom link: LIO-SAM has
    # no localization-only mode, so this run still does live local SLAM (below) into
    # a fresh "odom" frame, while this node ICP-registers that against the
    # previously saved map cloud and publishes the corrected map->odom.
    map_localizer_node = Node(
        package='lio_sam_bringup',
        executable='pcd_map_localizer',
        name='pcd_map_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_pcd_path': map_pcd_path,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'scan_topic': 'ouster_points',
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_z': initial_z,
            'initial_yaw': initial_yaw,
        }],
    )

    image_projection_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )
    feature_extraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )
    imu_preintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )
    map_optimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([lio_sam_share, 'config', 'rviz2.rviz'])],
        condition=IfCondition(rviz_use),
        parameters=[{'use_sim_time': True}],
    )

    # Same settle-delay reasoning as lio_sam_slam_launch.py.
    delayed_slam = TimerAction(
        period=20.0,
        actions=[map_localizer_node, adapter_node, image_projection_node, feature_extraction_node,
                 imu_preintegration_node, map_optimization_node],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_map_pcd_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_z_cmd)
    ld.add_action(declare_initial_yaw_cmd)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_slam)
    ld.add_action(rviz_node)

    return ld
