"""機体の制御を起動し、設定に応じて首振り2D LiDARを有効にする。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_nodding_lidar = LaunchConfiguration('enable_nodding_lidar')
    lidar_frame = LaunchConfiguration('lidar_frame')

    mirs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mirs'), 'launch', 'mirs_launch.py']),
        ),
        launch_arguments={
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'lidar_frame': lidar_frame,
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # lidar_launch.py owns the driver when nodding mode is enabled.
            'enable_lidar': PythonExpression([
                "'", LaunchConfiguration('enable_lidar'), "' == 'true' and '",
                enable_nodding_lidar, "' == 'false'",
            ]),
            'scan_safety_stop_distance': LaunchConfiguration('scan_safety_stop_distance'),
            'scan_safety_resume_distance': LaunchConfiguration('scan_safety_resume_distance'),
            'scan_safety_forward_angle': LaunchConfiguration('scan_safety_forward_angle'),
            'enable_parameter_publisher': LaunchConfiguration('enable_parameter_publisher'),
            'enable_micro_ros': LaunchConfiguration('enable_micro_ros'),
            # The EKF provides odom -> base_footprint for scan deskewing and
            # is the sole local-odometry TF publisher in this bringup.
            'enable_ekf_local': 'true',
            'enable_static_base_link_to_imu_link_tf': 'true',
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'enable_nav2': LaunchConfiguration('enable_nav2'),
            # RTAB-Map replaces slam_toolbox here to avoid competing map->odom TFs.
            'enable_slam_toolbox': 'false',
            'enable_static_base_link_to_laser_tf': PythonExpression([
                "'", enable_nodding_lidar, "' == 'false'",
            ]),
        }.items(),
    )

    nodding_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'launch', 'lidar_launch.py']),
        ),
        launch_arguments={
            'enable_nodding_lidar': enable_nodding_lidar,
            'servo_serial_port': LaunchConfiguration('servo_angle_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'lidar_frame': lidar_frame,
            'enable_lidar': LaunchConfiguration('enable_lidar'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'fixed_frame': 'odom',
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'base_to_servo_z': LaunchConfiguration('base_to_servo_z'),
        }.items(),
        condition=IfCondition(enable_nodding_lidar),
    )

    matching_cloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nodding_lidar'), 'launch', 'matching_cloud_launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'motion_cloud_config': LaunchConfiguration('motion_cloud_config'),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", enable_nodding_lidar, "' == 'true' and '",
            LaunchConfiguration('enable_rtabmap'), "' == 'true'",
        ])),
    )
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", enable_nodding_lidar, "' == 'true' and '",
            LaunchConfiguration('enable_rtabmap'), "' == 'true'",
        ])),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'rtabmap.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('scan_cloud', '/points_matching'),
            ('odom', '/odometry/local'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_nodding_lidar', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap', default_value='false'),
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution([
                FindPackageShare('mirs'), 'maps', 'es.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'motion_cloud_config', default_value=PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'motion_cloud.yaml'])),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('scan_safety_stop_distance', default_value='0.35'),
        DeclareLaunchArgument('scan_safety_resume_distance', default_value='0.50'),
        DeclareLaunchArgument('scan_safety_forward_angle', default_value='30.0'),
        DeclareLaunchArgument('enable_parameter_publisher', default_value='true'),
        DeclareLaunchArgument('enable_micro_ros', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='false'),
        DeclareLaunchArgument('base_to_servo_z', default_value='0.142'),
        mirs,
        nodding_lidar,
        matching_cloud,
        rtabmap,
    ])
