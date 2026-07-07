import os
from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_description')
    world_file = path.join(pkg_share, 'worlds', 'house.world')
    xacro_file = path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    model_path = path.join(pkg_share, 'models')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_sim_resource_value = model_path + (os.pathsep + existing_resource_path if existing_resource_path else '')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_sim_resource_value,
    )

    existing_gz = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path_value = model_path + (os.pathsep + existing_gz if existing_gz else '')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path_value,
    )

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    gz_server = GzServer(
        world_sdf_file=world_file,
        verbosity_level='4',
    )

    def _sanitize_snap_paths(value: str) -> str:
        return ':'.join(
            part for part in value.split(':')
            if '/snap/' not in part and 'snap/code' not in part
        )

    gui_env = os.environ.copy()
    if 'LD_LIBRARY_PATH' in gui_env:
        gui_env['LD_LIBRARY_PATH'] = _sanitize_snap_paths(gui_env['LD_LIBRARY_PATH'])
    if 'PATH' in gui_env:
        gui_env['PATH'] = _sanitize_snap_paths(gui_env['PATH'])
    if 'XDG_DATA_DIRS' in gui_env:
        gui_env['XDG_DATA_DIRS'] = _sanitize_snap_paths(gui_env['XDG_DATA_DIRS'])
    if 'XDG_CONFIG_DIRS' in gui_env:
        gui_env['XDG_CONFIG_DIRS'] = _sanitize_snap_paths(gui_env['XDG_CONFIG_DIRS'])
    for key in list(gui_env):
        if key.startswith('SNAP') or 'VSCODE_SNAP' in key or key == 'LD_PRELOAD':
            gui_env.pop(key, None)
        elif key in {
            'GTK_PATH',
            'GIO_MODULE_DIR',
            'GTK_EXE_PREFIX',
            'GTK_IM_MODULE_FILE',
        }:
            gui_env.pop(key, None)
        elif key == 'XDG_DATA_HOME' and gui_env.get(key, '').startswith('/home') and '/snap/' in gui_env[key]:
            gui_env.pop(key, None)

    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        env=gui_env,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_robot',
        output='screen',
        arguments=[
            '-world', 'default',
            '-string', Command(['xacro ', xacro_file]),
            '-name', 'diff_robot',
            '-allow_renaming', 'true',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
    )

    spawn_robot_after_delay = TimerAction(
        period=3.0,
        actions=[spawn_robot_node],
    )

    # Neu GUI mo truoc khi robot duoc spawn, no se khong tu dong nhan entity moi
    # (gioi han cua gz-sim), khien robot khong hien trong Gazebo. Nen chi mo GUI
    # sau khi spawn_robot_node da chay xong.
    launch_gui_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[gz_client],
        )
    )

    return LaunchDescription([
        set_gz_sim_resource_path,
        set_gazebo_model_path,
        gz_server,
        robot_state_publisher_node,
        spawn_robot_after_delay,
        launch_gui_after_spawn,
    ])
