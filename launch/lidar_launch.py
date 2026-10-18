"""
Launch the servo and point cloud nodes.

LiDAR自体 (sllidar_ros2) は、既存の起動ファイルで起動すること。
sllidar_ros2 の frame_id は、このパッケージの child_frame (lidar_link) に合わせること。

使い方の例:
  ros2 launch nodding_lidar lidar_launch.py use_hardware:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('nodding_lidar'), 'config', 'nodding_lidar.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_nodding_lidar', default_value='true',
            description='首振りLiDARと点群生成ノードを起動する'),
        DeclareLaunchArgument(
            'use_pca9685', default_value='false',
            description='true にするとシリアル入力の代わりにPCA9685サーボ指令ノードを使う'),
        DeclareLaunchArgument('servo_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument(
            'publish_base_to_servo', default_value='true',
            description='base_link -> servo_link の静的TFを配信する'),
        DeclareLaunchArgument('base_to_servo_x', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_y', default_value='0.0'),
        DeclareLaunchArgument(
            'base_to_servo_z', default_value='0.12945',
            description='既存LiDAR高さ0.1745mとセンサオフセット0.04505mからの暫定値'),
        DeclareLaunchArgument('base_to_servo_roll', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='false にすると、サーボを動かさず、角度とTFだけ出す'),

        Node(
            package='nodding_lidar',
            executable='serial_angle_bridge',
            name='serial_angle_bridge',
            output='screen',
            parameters=[params, {
                'serial_port': LaunchConfiguration('servo_serial_port'),
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_nodding_lidar'),
                "' == 'true' and '", LaunchConfiguration('use_pca9685'), "' == 'false'",
            ])),
        ),

        Node(
            package='nodding_lidar',
            executable='servo_tf_node',
            name='servo_tf_node',
            output='screen',
            parameters=[params, {
                'child_frame': LaunchConfiguration('lidar_frame'),
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_nodding_lidar'),
                "' == 'true' and '", LaunchConfiguration('use_pca9685'), "' == 'false'",
            ])),
        ),

        Node(
            package='nodding_lidar',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                params,
                {'use_hardware': ParameterValue(
                    LaunchConfiguration('use_hardware'), value_type=bool)},
                {'child_frame': LaunchConfiguration('lidar_frame')},
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_nodding_lidar'),
                "' == 'true' and '", LaunchConfiguration('use_pca9685'), "' == 'true'",
            ])),
        ),

        Node(
            package='nodding_lidar',
            executable='scan_to_cloud',
            name='scan_to_cloud',
            output='screen',
            parameters=[params],
            condition=IfCondition(LaunchConfiguration('enable_nodding_lidar')),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_servo_static_tf',
            arguments=[
                '--x', LaunchConfiguration('base_to_servo_x'),
                '--y', LaunchConfiguration('base_to_servo_y'),
                '--z', LaunchConfiguration('base_to_servo_z'),
                '--roll', LaunchConfiguration('base_to_servo_roll'),
                '--pitch', LaunchConfiguration('base_to_servo_pitch'),
                '--yaw', LaunchConfiguration('base_to_servo_yaw'),
                '--frame-id', 'base_link', '--child-frame-id', 'servo_link',
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_nodding_lidar'),
                "' == 'true' and '", LaunchConfiguration('publish_base_to_servo'),
                "' == 'true'",
            ])),
        ),
    ])
