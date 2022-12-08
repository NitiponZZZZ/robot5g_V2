from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s2_launch.py'])

    return LaunchDescription([
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_teensy",
            arguments=['serial', '--dev', '/dev/esp32'],
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='core',
            name='robot_core',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
        Node(
            package='robot5g',
            executable='power',
            name='robot_feedback',
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='data_pub',
            name='DATA_Sender',
            output='screen',
        ),
        Node(
            package='robot5g',
            executable='data_sub',
            name='DATA_Reader',
            output='screen',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='both',
        ),

        Node(
            package='robot5g',
            executable='joystick',
            name='joy_teleop',
            output='both',
        ),
    ])
