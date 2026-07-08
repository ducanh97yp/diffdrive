#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Same LAN cross-talk concern as the other launch files here - must match the
    # domain mola_localize_launch.py / rviz_joint_control.launch.py run on.
    os.environ['ROS_DOMAIN_ID'] = '161'

    # Cau hinh/mac dinh: file params nam trong package sau khi build/install,
    # ban do 2D lay tu pointcloud_to_occupancygrid.py + map_saver_cli (xem nav2_params.yaml).
    pkg_share = get_package_share_directory('mola_bringup')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.expanduser('~/ws_ros2_test/maps/mymap_2d.yaml')

    # Cho phep override map/params/use_rviz tu command line khi ros2 launch.
    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    params_arg = DeclareLaunchArgument('params_file', default_value=default_params_file)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Danh sach node duoc lifecycle_manager quan ly (configure + activate theo thu tu nay).
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server',
                        'behavior_server', 'bt_navigator']

    # No amcl here on purpose: MOLA (mola_localize_launch.py) must already be running
    # and providing map->odom. This launch only adds costmaps/planning/control on top.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
    )

    # Sinh duong di toan cuc (global path) tren costmap toan cuc, dung NavfnPlanner (xem params).
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    # Bam theo duong di (FollowPath/DWB) va sinh lenh /cmd_vel dua tren costmap cuc bo.
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    # Xu ly cac hanh vi phuc hoi (spin, back_up, wait...) khi robot bi ket/mat duong.
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    # Behavior Tree dieu phoi toan bo qua trinh navigate_to_pose (goi planner/controller/behavior).
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    # Quan ly vong doi (configure -> activate) cho toan bo lifecycle_nodes o tren theo dung thu tu.
    # autostart=True nen khong can goi ros2 lifecycle set thu cong.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': lifecycle_nodes}],
    )

    # Dung san rviz config mac dinh cua nav2_bringup (co panel Nav2 Goal, costmap, path...).
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_rviz_arg,
        map_server_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
        rviz_node,
    ])
