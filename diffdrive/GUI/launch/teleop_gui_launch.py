#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Same LAN cross-talk concern as the other launch files in this workspace - keep
    # domain consistent so this node sees the same graph as Gazebo/fast_lio.
    os.environ['ROS_DOMAIN_ID'] = '161'

    teleop_gui_node = Node(
        package='GUI',
        executable='teleop_gui.py',
        name='teleop_gui',
        output='screen',
        parameters=[{
            'linear_speed': 0.5,
            'angular_speed': 1.0,
        }],
    )

    return LaunchDescription([teleop_gui_node])
