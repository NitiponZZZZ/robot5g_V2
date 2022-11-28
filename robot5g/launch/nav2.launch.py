from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    map_dir = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution(
            [FindPackageShare('robot5g'), 'maps', 'building7and6dull.yaml'])
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [FindPackageShare('robot5g'), 'config', 'nav2_params.yaml'])
    )
    nav2_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch'])

    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Path of map'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),


    ])
