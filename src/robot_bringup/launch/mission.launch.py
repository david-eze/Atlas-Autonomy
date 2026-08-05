import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='office.world')

    bringup_dir = get_package_share_directory('robot_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'world': world}.items()
    )

    safety_node = Node(
        package='robot_safety',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    perception_node = Node(
        package='robot_perception',
        executable='detection_node',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    recovery_node = Node(
        package='robot_mission',
        executable='recovery_manager',
        name='recovery_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    semantic_nav_node = Node(
        package='robot_mission',
        executable='semantic_navigation',
        name='semantic_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf2_diagnostics_node = Node(
        package='robot_tf2_diagnostics',
        executable='tf2_diagnostics_node',
        name='tf2_diagnostics',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    web_dashboard_node = Node(
        package='robot_mission',
        executable='web_dashboard',
        name='web_dashboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='office.world'),
        sim_launch,
        safety_node,
        perception_node,
        recovery_node,
        semantic_nav_node,
        tf2_diagnostics_node,
        web_dashboard_node
    ])
