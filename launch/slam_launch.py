"""機体の制御とEKFによる位置推定を起動し、首振り2D LiDARの点群でRTAB-Mapの地図を作る。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mirs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mirs'), 'launch', 'mirs_launch.py'])),
        launch_arguments={
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'false',
            'enable_ekf_local': 'true',
            'enable_static_base_link_to_laser_tf': 'false',
            'enable_static_base_link_to_imu_link_tf': 'true',
            'enable_rviz': 'false',
            'enable_nav2': 'false',
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
    matching_cloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nodding_lidar'), 'launch', 'matching_cloud_launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'motion_cloud_config': LaunchConfiguration('motion_cloud_config'),
        }.items(),
    )
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen', parameters=[
            PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'rtabmap.yaml']),
            {'use_sim_time': use_sim_time, 'database_path': database_path},
        ], remappings=[('scan_cloud', '/points_matching'), ('odom', '/odometry/local')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rtabmap_3d_rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('nodding_lidar'), 'rviz', 'rtabmap_3d.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'motion_cloud_config', default_value=PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'motion_cloud.yaml'])),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), '.ros', 'mirs_3d.db']),
            description='RTAB-Map 3D mapping database'),
        GroupAction(actions=[mirs], scoped=True),
        # Keep the child RViz override from disabling the SLAM viewer.
        GroupAction(actions=[nodding], scoped=True),
        matching_cloud,
        rtabmap,
        rviz,
    ])
