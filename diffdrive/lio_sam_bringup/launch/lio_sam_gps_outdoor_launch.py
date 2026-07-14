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

    world = LaunchConfiguration('world')
    rviz_use = LaunchConfiguration('rviz')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='smalltown.world',
        description='Gazebo world file (from diff_description/worlds)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch LIO-SAM RViz view'
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_description_share, 'launch', 'rviz_joint_control.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    params_file = PathJoinSubstitution([lio_sam_bringup_share, 'config', 'lio_sam_params.yaml'])
    navsat_params_file = PathJoinSubstitution(
        [lio_sam_bringup_share, 'config', 'navsat_transform_outdoor.yaml'])
    ekf_monitoring_params_file = PathJoinSubstitution(
        [lio_sam_bringup_share, 'config', 'ekf_monitoring.yaml'])

    adapter_node = Node(
        package='lio_sam_bringup',
        executable='gz_lidar_to_ouster',
        name='gz_lidar_to_ouster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Same architecture as lio_sam_slam_launch.py's SLAM mode - map->odom stays a
    # static identity link. Outdoor mode does not add a second TF-owning node;
    # GPS instead feeds mapOptmization's own GPSFactor (see navsat_transform_node
    # below) as a sparse pose-graph correction, same map->odom edge as indoors.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
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

    # Converts /gps/fix (lat/lon) -> a local ENU odometry estimate, self-calibrated
    # against LIO-SAM's own /lio_sam/mapping/odometry (see navsat_transform_outdoor.yaml
    # for why) so its output lands in the same "odom" frame LIO-SAM's pose graph
    # already uses. Publishes only a topic, never a TF - broadcast_utm_transform is
    # off. Output is remapped straight onto lio_sam_params.yaml's existing gpsTopic
    # ("odometry/gpsz"), which mapOptmization.cpp's dormant addGPSFactor() already
    # consumes - no LIO-SAM code changes needed.
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params_file, {'use_sim_time': True}],
        remappings=[
            ('gps/fix', '/gps/fix'),
            ('imu', '/imu'),
            ('odometry/filtered', '/lio_sam/mapping/odometry'),
            ('odometry/gps', 'odometry/gpsz'),
        ],
    )

    # Monitoring-only EKF - NOT part of the navigation-critical pose estimate
    # (LIO-SAM's own GPSFactor already handles that, unchanged). publish_tf: false
    # (see ekf_monitoring.yaml) means this never broadcasts map->odom, so it can't
    # conflict with map_to_odom_tf above. Output remapped to a distinct topic name
    # to avoid any confusion with navsat_transform_node's own odometry/filtered
    # input (which is remapped to /lio_sam/mapping/odometry directly, not read
    # from an EKF).
    ekf_monitoring_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_monitoring_params_file, {'use_sim_time': True}],
        remappings=[
            ('odometry/wheel', '/lio_sam/mapping/odometry'),
            ('odometry/gps', '/odometry/gpsz'),
            ('imu/data', '/imu'),
            ('odometry/filtered', '/monitoring/global_odometry'),
        ],
    )

    # Converts the monitoring EKF's smoothed position back to lat/lon (via
    # navsat_transform_node's /toLL) for external GIS/dashboard consumption and
    # for AerialMap - see rviz_outdoor.rviz, which reads /monitoring/gps_fix
    # instead of the raw (noisier) /gps/fix.
    gps_fix_from_odom_node = Node(
        package='lio_sam_bringup',
        executable='gps_fix_from_odom.py',
        name='gps_fix_from_odom',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # rviz_outdoor.rviz (this package, not LIO-SAM's own rviz2.rviz used indoors)
    # adds a rviz_satellite/AerialMap display anchored to /gps/fix, so goals can
    # be picked on real satellite imagery instead of a blank local-frame canvas -
    # see README's outdoor-routing section.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([lio_sam_bringup_share, 'config', 'rviz_outdoor.rviz'])],
        condition=IfCondition(rviz_use),
        parameters=[{'use_sim_time': True}],
    )

    # Same settle-delay reasoning as lio_sam_slam_launch.py - starting SLAM before the
    # robot finishes its physics-drop settle causes bad IMU gravity/bias
    # initialization (gtsam::IndeterminantLinearSystemException crash in
    # imuPreintegration). Outdoor worlds are heavier to load (more models,
    # navsat system plugin) than house.world, so spawn/physics-settle itself
    # takes longer here - 20s occasionally wasn't enough (confirmed: crash
    # reproduced even with 20s once system load was otherwise clean). Bumped
    # to 35s for more margin; still cheap relative to a full test run.
    delayed_slam = TimerAction(
        period=35.0,
        actions=[map_to_odom_tf, adapter_node, image_projection_node, feature_extraction_node,
                 imu_preintegration_node, map_optimization_node, navsat_transform_node,
                 ekf_monitoring_node, gps_fix_from_odom_node],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_slam)
    ld.add_action(rviz_node)

    return ld
