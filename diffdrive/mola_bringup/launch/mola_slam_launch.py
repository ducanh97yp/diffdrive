#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def _sanitized_env():
    """rviz2 (spawned deep inside mola_lidar_odometry's launch file) inherits this
    process's environment as-is. When that process runs under a snap-packaged VS Code /
    Claude Code session, stray SNAP_*/GTK_*/GIO_* vars leak in and make the dynamic
    linker resolve libpthread.so.0 from /snap/core20/... instead of the system one,
    crashing rviz2 with "symbol lookup error: undefined symbol: __libc_pthread_init".
    Same class of issue diff_description/launch/gazebo.launch.py works around for its
    own gz_client - strip the same vars here.
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
    # rviz_joint_control.launch.py / mola_localize_launch.py to talk to the same graph.
    os.environ['ROS_DOMAIN_ID'] = '161'

    # input_topic: robot's simulated Mid-360 publishes PointCloud2 on /points (see
    # diff_description/urdf/lidar.xacro), not the real-sensor default /livox/lidar.
    # min_intensity: gz-sim's gpu_lidar sensor doesn't simulate reflectivity, so every
    # point has intensity=0.0 - the default min_intensity=1.0 would filter out 100% of
    # points and starve MOLA of any data.
    # to save map: ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/home/andy1/ws_ros2_test/maps/ten_map_ban_muon'"
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

    # pass_through_filter_node = Node(
    #     package='mola_bringup',
    #     executable='intensity_passthrough_filter.py',
    #     name='intensity_passthrough_filter',
    #     output='screen'
    # )

    # NOTE: the official ros-jazzy-mola-lidar-odometry apt package only ships
    # ros2-lidar-odometry.launch.py - the "-katana" variant was the original repo
    # author's unpublished personal fork and doesn't exist in this install.
    slam_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry" ,
             'ros2-lidar-odometry.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'start_mapping_enabled:=true',
             'start_active:=true'],
        output='screen',
        env=_sanitized_env(),
    )
    
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(filterpass)
    # ld.add_action(pass_through_filter_node)
    ld.add_action(slam_cmd)
    ld.add_action(plot_node)

    return ld
# def main(args=None):
#    generate_launch_description()

# if __name__ == '__main__':
#    main()