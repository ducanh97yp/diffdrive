from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = FindPackageShare('diff_description')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'diff_description.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'my_controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'robot.rviz'])

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    gz_server = GzServer(
        world_sdf_file=world_file,
        verbosity_level='4',
    )

    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'source_list': ['/joint_broad/joint_states'],
                'rate': 30,
                'use_sim_time': True,
            }
        ],
    )

    cmd_vel_stamper_node = Node(
        package='diff_description',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
    )

    delayed_spawn_robot = TimerAction(
        period=3.0,
        actions=[spawn_robot_node],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_broad',
        output='screen',
        arguments=['joint_broad', '--controller-manager-timeout', '60'],
    )

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

    spawn_joint_broad_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_broad_spawner],
        )
    )

    spawn_diff_cont_after_joint_broad = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_cont_spawner],
        )
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_server,
        gz_client,
        clock_bridge_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        cmd_vel_stamper_node,
        delayed_spawn_robot,
        spawn_joint_broad_after_robot,
        spawn_diff_cont_after_joint_broad,
        #joint_state_publisher_gui_node,
        rviz_node,
    ])
