from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = FindPackageShare('diff_description')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'diff_description.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Gazebo server with world file
    gz_server = GzServer(
        world_sdf_file=world_file,
        verbosity_level='4',
    )

    # Gazebo client GUI
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_server,
        gz_client,
        robot_state_publisher_node,
    ])
