"""Start the nodding LiDAR stack and a small RViz PointCloud2 view."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nodding_share = FindPackageShare('nodding_lidar')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            nodding_share, 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'enable_nodding_lidar': 'true',
            'enable_rtabmap': 'false',
            'enable_nav2': 'false',
            'enable_rviz': 'false',
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'servo_angle_port': LaunchConfiguration('servo_angle_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='nodding_pointcloud_rviz',
        arguments=['-d', PathJoinSubstitution([
            nodding_share, 'rviz', 'pointcloud.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen', condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_angle_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        bringup,
        rviz,
    ])
