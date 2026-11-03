"""LiDARの点群同士を照合して移動量を求め、地図を作る。機体の制御やESP32接続は使用しない。"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
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
            # No external odometry: avoid a points -> odom -> points cycle.
            'fixed_frame': 'base_link',
            'enable_rviz': 'false',
        }.items(),
    )
    odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
        output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_imu_to_init': False,
            'deskewing': False,
            # Each input is a thin scan; do not require surface normals.
            'Icp/PointToPlane': 'false',
            'Icp/VoxelSize': '0.05',
            'OdomF2M/ScanMaxSize': '15000',
        }],
        remappings=[('scan_cloud', '/points'), ('odom', '/odometry/lidar')],
    )
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen', parameters=[
            PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'rtabmap.yaml']),
            {'use_sim_time': use_sim_time, 'database_path': database_path,
             'Icp/PointToPlane': 'false'},
        ], remappings=[('scan_cloud', '/points'), ('odom', '/odometry/lidar')],
    )
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
        output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'subscribe_odom_info': True,
            'odometry_node_name': 'icp_odometry',
            'approx_sync': True,
            'wait_for_transform': 0.2,
        }],
        remappings=[('scan_cloud', '/points'), ('odom', '/odometry/lidar')],
        condition=IfCondition(LaunchConfiguration('enable_rtabmap_viz')),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rtabmap_3d_rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('nodding_lidar'), 'rviz', 'rtabmap_3d.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "90"),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='true'),
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), '.ros', 'nodding_standalone.db']),
            description='RTAB-Map 3D mapping database'),
        # Keep the child RViz override from disabling the SLAM viewer.
        GroupAction(actions=[nodding], scoped=True),
        odometry,
        rtabmap,
        rtabmap_viz,
        rviz,
    ])
