from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([
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
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_teensy",
            arguments=['serial', '--dev', '/dev/ttyUSB0'],  # /dev/esp32
            output='screen',
        ),
    ])
