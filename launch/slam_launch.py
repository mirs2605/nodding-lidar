"""Robot, EKF and RTAB-Map SLAM using the nodding LiDAR point cloud."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'database_path': database_path,
            'frame_id': 'base_link', 'odom_frame_id': '',
            'odom_frame_id_init': 'odom',
            'map_frame_id': 'map', 'subscribe_odom': True,
            'subscribe_scan_cloud': True, 'subscribe_rgb': False,
            'subscribe_depth': False, 'subscribe_rgbd': False,
            'approx_sync': True, 'wait_for_transform': 0.2,
            'Reg/Strategy': '1', 'Reg/Force3DoF': 'false',
            'Grid/FromDepth': 'false', 'Grid/3D': 'true',
        }], remappings=[('scan_cloud', '/points'), ('odom', '/odometry/local')],
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
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), '.ros', 'mirs_3d.db']),
            description='RTAB-Map 3D mapping database'),
        mirs,
        nodding,
        rtabmap,
        rviz,
    ])
