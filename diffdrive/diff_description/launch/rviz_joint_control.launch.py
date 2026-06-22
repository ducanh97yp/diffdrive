from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    # Lay duong dan trong package sau khi da build/install.
    pkg_share = FindPackageShare('diff_description')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'diff_description.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'my_controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'robot.rviz'])

    # robot_state_publisher nhan robot_description dang URDF, nen can chay xacro truoc.
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Gazebo Sim server chay physics/world; client la cua so GUI.
    gz_server = GzServer(
        world_sdf_file=world_file,
        verbosity_level='4',
    )

    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    # Dong bo thoi gian Gazebo -> ROS. Quan trong khi cac node dung use_sim_time.
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Publish TF cho cac link cua robot tu robot_description va /joint_states.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # RViz can /joint_states de hien cac joint dong nhu 2 banh xe.
    # joint_broad publish topic rieng /joint_broad/joint_states, node nay merge ra /joint_states.
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
            '-topic', 'robot_description',
            '-name', 'diff_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
    )

    # Cho Gazebo va robot_state_publisher khoi dong truoc khi spawn model.
    delayed_spawn_robot = TimerAction(
        period=3.0,
        actions=[spawn_robot_node],
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

    # Chi spawn controller sau khi Gazebo da tao xong robot, tranh loi khong thay hardware.
    spawn_joint_broad_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_broad_spawner],
        )
    )

    # Kich hoat diff drive sau joint_state_broadcaster de RViz co joint state som.
    spawn_diff_cont_after_joint_broad = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_cont_spawner],
        )
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
