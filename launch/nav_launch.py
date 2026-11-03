"""機体の制御、EKFによる位置推定、首振り2D LiDAR、保存済み地図を使うNav2を起動する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mirs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mirs'), 'launch', 'mirs_launch.py'])),
        launch_arguments={
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'false',
            'enable_ekf_local': 'true',
            'enable_rviz': 'true',
            'enable_nav2': 'true',
            'enable_slam_toolbox': 'false',
        }.items(),
    )
    nodding = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nodding_lidar'), 'launch', 'lidar_launch.py'])),
        launch_arguments={
            'enable_nodding_lidar': 'true',
            'servo_serial_port': LaunchConfiguration('servo_angle_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'enable_lidar': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'fixed_frame': 'odom',
            'enable_rviz': 'false',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('mirs'), 'maps', 'es.yaml'])),
        mirs,
        nodding,
    ])
