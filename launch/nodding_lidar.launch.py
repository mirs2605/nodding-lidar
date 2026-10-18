"""
Launch the servo and point cloud nodes.

LiDAR自体 (sllidar_ros2) は、既存の起動ファイルで起動すること。
sllidar_ros2 の frame_id は、このパッケージの child_frame (lidar_link) に合わせること。

使い方の例:
  ros2 launch nodding_lidar nodding_lidar.launch.py use_hardware:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('nodding_lidar'), 'config', 'nodding_lidar.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='false にすると、サーボを動かさず、角度とTFだけ出す'),
        DeclareLaunchArgument(
            'publish_base_to_servo', default_value='false',
            description='true にすると、base_link -> servo_link の静的TFを出す'
                        ' (既存のURDF等で出している場合は false のまま)'),

        Node(
            package='nodding_lidar',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                params,
                {'use_hardware': ParameterValue(
                    LaunchConfiguration('use_hardware'), value_type=bool)},
            ],
        ),

        Node(
            package='nodding_lidar',
            executable='scan_to_cloud',
            name='scan_to_cloud',
            output='screen',
            parameters=[params],
        ),

        # base_link -> servo_link (固定)。値は取り付け位置に合わせて変更すること。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_servo_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'servo_link',
            ],
            condition=IfCondition(LaunchConfiguration('publish_base_to_servo')),
        ),
    ])
