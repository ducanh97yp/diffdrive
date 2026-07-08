#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch_ros.actions import Node

def _sanitized_env():
    """See mola_slam_launch.py - strips SNAP_*/GTK_*/GIO_* env leakage that crashes
    rviz2 (spawned inside mola_lidar_odometry's launch file) with a symbol lookup
    error when this process is itself running under a snap-packaged VS Code session.
    """
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('SNAP') or key in {
            'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
            'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR',
            'GSETTINGS_SCHEMA_DIR', 'GIO_MODULE_DIR', 'LOCPATH',
        }:
            env.pop(key, None)
        elif key == 'XDG_DATA_HOME' and '/snap/' in env.get(key, ''):
            env.pop(key, None)
    return env


def generate_launch_description():
    # This machine shares a LAN with other machines that also default to
    # ROS_DOMAIN_ID=0, so /tf, /robot_description, /joint_states leak in from their
    # nodes (mismatched meshes, TF flapping -> RViz's PointCloud2 Transform status
    # blinks red). Pin a distinct domain (LAN IP's last octet, unlikely to collide) so
    # this launch's DDS traffic doesn't mix with theirs. Must match the value used in
    # rviz_joint_control.launch.py / mola_slam_launch.py to talk to the same graph.
    os.environ['ROS_DOMAIN_ID'] = '161'

    # NOTE: the paths below must point to a map of the environment the robot is
    # actually running in. The FIBO.mm/masterFIBO.mm maps bundled in this repo were
    # captured on the real FIBO building with a real Mid-360 - they are NOT a map of
    # the Gazebo "house" world, so localization against them will not converge here.
    # Build your own map first with mola_slam_launch.py, save it via:
    #   ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/home/andy1/ws_ros2_test/maps/gazebo_house'"
    # which creates gazebo_house.mm/.simplemap at the paths below.
    mm_map = '/home/andy1/ws_ros2_test/maps/myroom.mm'
    simple_map = '/home/andy1/ws_ros2_test/maps/myroom.simplemap'

    # NOTE: same as mola_slam_launch.py - the official apt package doesn't ship a
    # separate "-localize-katana" launch file. The unified ros2-lidar-odometry.launch.py
    # handles both mapping and localization via start_mapping_enabled/enable_mapping and
    # the mola_initial_map_*_file args passed below.
    localize_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry" ,
             'ros2-lidar-odometry.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'start_mapping_enabled:=false',
             'start_active:=false',
             f"mola_initial_map_mm_file:={mm_map}",
             f"mola_initial_map_sm_file:={simple_map}"],
        output='screen',
        env=_sanitized_env(),
    )
    
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    # See mola_slam_launch.py for why these overrides are needed (topic name + intensity).
    filterpass = Node(
        package='mola_bringup',
        executable='filterpass.py',
        name='filterpass',
        parameters=[{
            'input_topic': '/points',
            'min_intensity': 0.0,
        }],
        output='screen'
    )


    
    ld = LaunchDescription()
    ld.add_action(filterpass)

    ld.add_action(localize_cmd)
    ld.add_action(plot_node)


    return ld

# def main(args=None):
#    generate_launch_description()

# if __name__ == '__main__':
#    main()
