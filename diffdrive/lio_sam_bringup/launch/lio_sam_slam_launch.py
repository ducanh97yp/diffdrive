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

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='house.world',
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

    adapter_node = Node(
        package='lio_sam_bringup',
        executable='gz_lidar_to_ouster',
        name='gz_lidar_to_ouster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # LIO-SAM folds pose-graph/loop-closure corrections directly into what it reports
    # as "odom" (there is no separate corrected "map" frame) - this identity map->odom
    # link is the same pattern LIO-SAM's own reference run.launch.py uses, so the TF
    # tree has a proper map root for Nav2's global costmap without LIO-SAM needing to
    # publish it itself.
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([lio_sam_share, 'config', 'rviz2.rviz'])],
        condition=IfCondition(rviz_use),
        parameters=[{'use_sim_time': True}],
    )

    # Same settle-delay reasoning as fast_lio's own bringup used to have: starting
    # SLAM immediately after spawn (while the robot is still settling from its
    # physics drop / before controllers activate) causes bad IMU gravity/bias
    # initialization - for LIO-SAM this showed up as a gtsam::IndeterminantLinear
    # SystemException crash in imuPreintegration on first launch. 20s clears it.
    delayed_slam = TimerAction(
        period=20.0,
        actions=[map_to_odom_tf, adapter_node, image_projection_node, feature_extraction_node,
                 imu_preintegration_node, map_optimization_node],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_slam)
    ld.add_action(rviz_node)

    return ld
