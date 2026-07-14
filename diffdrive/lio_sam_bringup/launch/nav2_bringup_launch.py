#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['ROS_DOMAIN_ID'] = '161'

    pkg_share = get_package_share_directory('lio_sam_bringup')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    default_graph_file = os.path.join(pkg_share, 'graphs', 'smalltown_demo_graph.geojson')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params_file)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    # Only consumed when params_file points route_server's bt_navigator tree at
    # navigate_w_route.xml (nav2_params_outdoor_route.yaml) - harmless otherwise.
    # Passed as a separate parameter override rather than baked into the yaml since
    # $(find-pkg-share ...) is a launch-XML substitution, not resolved when a plain
    # YAML file is loaded directly as a node's params file.
    graph_file_arg = DeclareLaunchArgument('graph_filepath', default_value=default_graph_file)
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    graph_filepath = LaunchConfiguration('graph_filepath')

    lifecycle_nodes = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    # route_server (nav2_route) is only exercised when params_file has a route_server:
    # block pointing bt_navigator's navigate_to_pose at navigate_w_route.xml (see
    # nav2_params_outdoor_route.yaml) - it's harmless to launch unconditionally
    # otherwise, since nothing calls its actions if the stock BT tree is in use.
    # Separate lifecycle manager so a route_server hiccup can't block the core
    # planner/controller/behavior/bt_navigator group from activating.
    route_lifecycle_nodes = ['route_server']

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': lifecycle_nodes}],
    )

    route_server_node = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        parameters=[params_file, {'graph_filepath': graph_filepath}],
    )

    lifecycle_manager_route_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_route',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': route_lifecycle_nodes}],
    )

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
        params_arg,
        use_rviz_arg,
        graph_file_arg,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
        route_server_node,
        lifecycle_manager_route_node,
        rviz_node,
    ])
