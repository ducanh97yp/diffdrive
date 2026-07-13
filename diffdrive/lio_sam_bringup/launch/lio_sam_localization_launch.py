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
    icp_update_period = LaunchConfiguration('icp_update_period')
    icp_max_correspondence_distance = LaunchConfiguration('icp_max_correspondence_distance')
    icp_fitness_score_threshold = LaunchConfiguration('icp_fitness_score_threshold')
    icp_voxel_leaf_scan = LaunchConfiguration('icp_voxel_leaf_scan')
    icp_degenerate_eigenvalue_ratio = LaunchConfiguration('icp_degenerate_eigenvalue_ratio')
    icp_max_correction_per_update = LaunchConfiguration('icp_max_correction_per_update')
    tf_tolerance = LaunchConfiguration('tf_tolerance')

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
    # Default tightened from pcd_map_localizer's own 1.0s node default: on short/steep
    # ramps (terrain_test.world), a full second between global-map ICP corrections is
    # too coarse a re-anchor cadence - LIO-SAM's own local odometry (imuPreintegration)
    # already tracks pitch continuously between corrections, but the correction itself
    # needs to catch up faster while the tilt is actively changing. Override higher
    # (e.g. 1.0) on flat maps to save CPU if 0.2s proves unnecessary there.
    declare_icp_update_period_cmd = DeclareLaunchArgument('icp_update_period', default_value='0.2')
    declare_icp_max_corr_dist_cmd = DeclareLaunchArgument('icp_max_correspondence_distance', default_value='1.0')
    declare_icp_fitness_threshold_cmd = DeclareLaunchArgument('icp_fitness_score_threshold', default_value='0.5')
    declare_icp_voxel_leaf_scan_cmd = DeclareLaunchArgument('icp_voxel_leaf_scan', default_value='0.2')
    # Tuned from real data on terrain_test.world's parallel-wall spawn corridor: a
    # genuinely weak translation axis there measured eigenvalue ratio ~0.30 against a
    # normally-constrained axis's ~0.67-0.79 - 0.45 sits with margin on both sides.
    # max_correction_per_update is a hard cap (independent of the eigenvalue check) on
    # how far a single ICP update may move map->odom - guards against the eigenvalue
    # check missing a large, noisy single-frame jump on a merely-weak (not fully
    # singular) axis, confirmed empirically to happen (~1.25m in one 0.2s update).
    declare_icp_degenerate_ratio_cmd = DeclareLaunchArgument('icp_degenerate_eigenvalue_ratio', default_value='0.45')
    declare_icp_max_correction_cmd = DeclareLaunchArgument('icp_max_correction_per_update', default_value='0.3')
    # AMCL-style forward-stamping on the map->odom broadcast (see pcd_map_localizer.cpp)
    # - confirmed necessary, not just precautionary: without it, controller_server hit
    # "Lookup would require extrapolation into the future" on map->odom, aborted
    # follow_path, and the robot never moved despite /cmd_vel still publishing.
    declare_tf_tolerance_cmd = DeclareLaunchArgument('tf_tolerance', default_value='0.2')

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
            'update_period': icp_update_period,
            'max_correspondence_distance': icp_max_correspondence_distance,
            'fitness_score_threshold': icp_fitness_score_threshold,
            'voxel_leaf_scan': icp_voxel_leaf_scan,
            'degenerate_eigenvalue_ratio': icp_degenerate_eigenvalue_ratio,
            'max_correction_per_update': icp_max_correction_per_update,
            'tf_tolerance': tf_tolerance,
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
    ld.add_action(declare_icp_update_period_cmd)
    ld.add_action(declare_icp_max_corr_dist_cmd)
    ld.add_action(declare_icp_fitness_threshold_cmd)
    ld.add_action(declare_icp_voxel_leaf_scan_cmd)
    ld.add_action(declare_icp_degenerate_ratio_cmd)
    ld.add_action(declare_icp_max_correction_cmd)
    ld.add_action(declare_tf_tolerance_cmd)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_slam)
    ld.add_action(rviz_node)

    return ld
