#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Same LAN cross-talk concern as the other launch files in this workspace - keep
    # domain consistent so this node sees the same graph as Gazebo/MOLA.
    os.environ['ROS_DOMAIN_ID'] = '161'

    # Run this alongside mola_localize_launch.py (needs its 'map' frame + filtered
    # point cloud already flowing) while driving the robot around to cover the space.
    # Once enough of the room has been driven past, save the result with:
    #   ros2 run nav2_map_server map_saver_cli -f /home/andy1/ws_ros2_test/maps/myroom_2d
    map_builder_node = Node(
        package='mola_bringup',
        executable='pointcloud_to_occupancygrid.py',
        name='pointcloud_to_occupancygrid',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar_filtered',
            'map_frame': 'map',
            'resolution': 0.05,
            'width_m': 40.0,
            'height_m': 40.0,
            'origin_x': -20.0,
            'origin_y': -20.0,
            'min_height': 0.05,
            'max_height': 0.6,
            'occ_hit_threshold': 2,
            'publish_rate_hz': 1.0,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([map_builder_node])
