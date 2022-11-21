from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('robot5g'), 'config', 'ekf.yaml'])

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
            executable='imu_publisher',
            name='imu_converter_node',
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
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='imu_tf_publisher',
        #     arguments=['0.1', '0.0', '0.05', '0.0', '0.0',
        #                '0.0', '1.0', 'base_link', 'imu_link'],
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[
        #         ekf_config_path
        #     ],
        #     remappings=[("odometry/filtered", "odom")]
        # ),
    ])
