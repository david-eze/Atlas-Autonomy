"""Launch the full robot simulation stack.

Starts Gazebo with the selected world, spawns the robot, and brings up
the sensor pipeline, EKF, safety monitor, and TF2 diagnostics.

Usage:
    ros2 launch robot_bringup simulation.launch.py environment:=office
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_description = FindPackageShare('robot_description')

    environment = LaunchConfiguration('environment')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    # Worlds live at the repo root. In the Docker container the repo is
    # mounted at /workspace/src/robot_autonomy; fall back to the package
    # share path for native installs.
    world_path = PathJoinSubstitution([
        '/workspace/src/robot_autonomy/worlds', environment + '.world',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='office',
                              choices=['office', 'warehouse', 'challenging']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),

        # Gazebo server (headless mode for CI/benchmarks).
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', world_path],
            output='screen',
            condition=IfCondition(headless),
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
            condition=IfCondition(LaunchConfiguration('headless').__eq__('false')),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ', PathJoinSubstitution([pkg_description, 'urdf', 'robot.urdf.xacro']),
                ]),
            }],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'autonomous_robot',
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
            ],
            output='screen',
        ),

        Node(
            package='robot_sensors',
            executable='lidar_filter',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robot_sensors'), 'config', 'lidar_filter.yaml'])],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robot_localization'), 'config', 'ekf.yaml'])],
        ),

        Node(
            package='robot_localization',
            executable='sensor_health_monitor',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robot_localization'), 'config', 'sensor_health_monitor.yaml'])],
        ),

        # Safety monitor (independent of Nav2).
        Node(
            package='robot_safety',
            executable='safety_monitor',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robot_safety'), 'config', 'safety.yaml'])],
        ),

        Node(
            package='robot_tf2_diagnostics',
            executable='tf2_diagnostics',
        ),
    ])
