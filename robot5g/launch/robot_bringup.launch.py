from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'),
         'launch', 'sllidar_s2_launch.py']
    )
    return LaunchDescription([

        Node(
            package='robot5g',
            executable='core',
            name='robot_core',
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='mqtt_pub',
            name='MQTT_Sender',
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='mqtt_sub',
            name='MQTT_Reader',
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='power',
            name='robot_feedback',
            output='screen',
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(lidar_launch_path)
        ),
    ])
