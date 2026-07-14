import os
from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                             SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    # May nay dung chung mang LAN voi cac may khac cung mac dinh ROS_DOMAIN_ID=0, nen
    # /tf, /robot_description, /joint_states bi lan sang node cua nguoi khac (thay mesh
    # la, TF nhay lien tuc -> RViz nhap nhay do o Transform cua PointCloud2). Gan domain
    # rieng (lay theo octet cuoi cua IP LAN cho it trung) de tach DDS multicast khoi ho.
    os.environ['ROS_DOMAIN_ID'] = '161'

    # world:=<filename under worlds/> lets test scenarios (e.g. terrain_test.world for
    # ramp/uneven-terrain testing) run without touching the default house.world.
    world_arg = DeclareLaunchArgument(
        'world', default_value='house.world',
        description='World SDF filename under diff_description/worlds/')

    # Lay duong dan trong package sau khi da build/install.
    pkg_share = get_package_share_directory('diff_description')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])
    xacro_file = path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Overridable so other bringup packages can supply their own controllers yaml
    # (e.g. a real-hardware variant that re-enables wheel odometry) without editing
    # this file.
    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=path.join(pkg_share, 'config', 'my_controllers.yaml'),
        description='Path to the ros2_control controllers yaml (diff_cont, joint_broad)')
    controller_config = LaunchConfiguration('controller_config')
    rviz_config = path.join(pkg_share, 'config', 'robot.rviz')
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

    # robot_state_publisher nhan robot_description dang URDF, nen can chay xacro truoc.
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Gazebo Sim server chay physics/world; client la cua so GUI.
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

    # Dong bo thoi gian Gazebo -> ROS. Quan trong khi cac node dung use_sim_time.
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Lidar 3D (kieu Mid-360) quet nhieu vong doc nen du lieu dung dang PointCloud2,
    # khong con la mot mat phang LaserScan don. Gz publish point cloud tren
    # "<topic>/points" (topic sensor la "scan" nen gz topic la "scan/points"),
    # remap sang /points cho gon.
    points_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_points_bridge',
        output='screen',
        arguments=['/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/scan/points', '/points')],
        parameters=[{'use_sim_time': True}],
    )

    # IMU tich hop san trong lidar Mid-360.
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
    )

    # GPS/navsat cho tu hanh outdoor. Chi co du lieu khi world dang load co
    # plugin gz-sim-navsat-system (vd outdoor.world) - vo hai voi cac world
    # indoor (house.world...) khong load plugin do, topic chi khong bao gio
    # co du lieu.
    gps_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_bridge',
        output='screen',
        arguments=['/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        remappings=[('/navsat', '/gps/fix')],
        parameters=[{'use_sim_time': True}],
    )

    # Publish TF cho cac link cua robot tu robot_description va /joint_states.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # teleop_twist_keyboard mac dinh publish Twist tren /cmd_vel.
    # diff_drive_controller tren Jazzy dang nhan TwistStamped, nen can node chuyen doi nay.
    cmd_vel_stamper_node = Node(
        package='diff_description',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Dua robot_description vao Gazebo thanh mot model co ten diff_robot.
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
            '-z', '0.5',
        ],
    )

    # Spawn the robot after a short delay so Gazebo can initialize.
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

    # joint_broad doc state interface cua cac joint va tao du lieu banh xe cho RViz.
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_broad',
        output='screen',
        arguments=['joint_broad', '--controller-manager-timeout', '60'],
    )

    # diff_cont nhan lenh van toc va dieu khien left_wheel_joint/right_wheel_joint.
    # Remap ~/cmd_vel sang /cmd_vel_stamped vi lenh da duoc dong dau thoi gian.
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_diff_cont',
        output='screen',
        arguments=[
            'diff_cont',
            '--param-file',
            controller_config,
            '--controller-manager-timeout',
            '60',
            '--controller-ros-args',
            '-r ~/cmd_vel:=/cmd_vel_stamped',
        ],
    )

    # Spawn the joint state broadcaster after a delay to ensure the robot exists.
    spawn_joint_broad_after_delay = TimerAction(
        period=6.0,
        actions=[joint_broad_spawner],
    )

    # Activate the diff drive controller after another short delay.
    spawn_diff_cont_after_delay = TimerAction(
        period=9.0,
        actions=[diff_cont_spawner],
    )

    # Node GUI nay chi can khi muon dieu khien joint bang slider, khong can khi chay Gazebo.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz dung config co Fixed Frame = odom de thay robot di chuyen theo odometry.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        env=gui_env,
    )

    return LaunchDescription([
        world_arg,
        controller_config_arg,
        set_gz_sim_resource_path,
        set_gazebo_model_path,
        gz_server,
        clock_bridge_node,
        points_bridge_node,
        imu_bridge_node,
        gps_bridge_node,
        robot_state_publisher_node,
        cmd_vel_stamper_node,
        spawn_robot_after_delay,
        launch_gui_after_spawn,
        spawn_joint_broad_after_delay,
        spawn_diff_cont_after_delay,
        #joint_state_publisher_gui_node,
        #rviz_node,
    ])
