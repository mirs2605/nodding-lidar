"""LiDARドライバ、サーボ制御、点群生成、RVizのノードを起動する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            description='首振り2D LiDARと点群生成ノードを起動する'),
        DeclareLaunchArgument(
            'use_pca9685', default_value='false',
            description='true にするとシリアル入力の代わりにPCA9685サーボ指令ノードを使う'),
        DeclareLaunchArgument('servo_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument(
            'fixed_frame', default_value='base_link',
            description='スキャン中の移動補正用フレーム（odomがない単体起動ではbase_link）'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'publish_base_to_servo', default_value='true',
            description='base_link -> servo_link の静的TFを配信する'),
        DeclareLaunchArgument('base_to_servo_x', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_y', default_value='0.0'),
        DeclareLaunchArgument(
            'base_to_servo_z', default_value='0.142',
            description='既存LiDAR高さ0.1745mとセンサオフセット0.0325mからの暫定値'),
        DeclareLaunchArgument('base_to_servo_roll', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_to_servo_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='false にすると、サーボを動かさず、角度とTFだけ出す'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='RViz2で /points を表示する'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s1_launch.py'])),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
                'frame_id': LaunchConfiguration('lidar_frame'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_lidar')),
        ),

        Node(
            package='nodding_lidar',
            executable='serial_angle_bridge',
            name='serial_angle_bridge',
            output='screen',
            parameters=[params, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
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
            parameters=[params, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'fixed_frame': LaunchConfiguration('fixed_frame'),
            }],
            condition=IfCondition(LaunchConfiguration('enable_nodding_lidar')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='nodding_pointcloud_rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'rviz', 'pointcloud.rviz'])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
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
