import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('robot_navigation'),
            'config', 'nav2_office.yaml'
        )
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=map_yaml_file,
        description='Full path to map file to load'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=params_file,
        description='Full path to Nav2 parameter file'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        nav2_bringup_launch
    ])
