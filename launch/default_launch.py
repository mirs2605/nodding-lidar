"""Robot, EKF and nodding LiDAR without SLAM, Nav2 or RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'launch',
                'bringup_launch.py'])),
            launch_arguments={
                'enable_nodding_lidar': 'true', 
                'enable_rtabmap': 'false',
                'enable_nav2': 'false', 'enable_rviz': 'false',
                'esp_port': LaunchConfiguration('esp_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'servo_angle_port': LaunchConfiguration('servo_angle_port'),
                'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
