"""Robot, EKF and RTAB-Map SLAM using the nodding LiDAR point cloud."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'base_link', 'odom_frame_id': 'odom',
            'map_frame_id': 'map', 'subscribe_odom': True,
            'subscribe_scan_cloud': True, 'subscribe_rgb': False,
            'subscribe_depth': False, 'subscribe_rgbd': False,
            'approx_sync': True, 'wait_for_transform': 0.2,
            'Reg/Strategy': '1', 'Grid/FromDepth': 'false',
        }], remappings=[('scan_cloud', '/points'), ('odom', '/odometry/local')],
    )
    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        mirs,
        nodding,
        rtabmap,
    ])
