"""独立した車輪・IMUオドメトリと単発点群から、旋回量を制限した照合用点群を作る。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """実機・シミュレーション共通の蓄積ノードを起動する定義を返す。

    cloud_topicはbase_link座標のスキャン内補償済み点群、odometry_topicは
    LiDARから独立した車輪・IMU推定。TF配信と機器への指令は行わない。
    """
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/points'),
        DeclareLaunchArgument('odometry_topic', default_value='/odometry/local'),
        DeclareLaunchArgument('matching_cloud_topic', default_value='/points_matching'),
        DeclareLaunchArgument(
            'motion_cloud_config', default_value=PathJoinSubstitution([
                FindPackageShare('nodding_lidar'), 'config', 'motion_cloud.yaml'])),
        Node(
            package='nodding_lidar', executable='motion_cloud_assembler',
            name='motion_cloud_assembler', output='screen',
            parameters=[LaunchConfiguration('motion_cloud_config'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('cloud', LaunchConfiguration('cloud_topic')),
                ('odometry', LaunchConfiguration('odometry_topic')),
                ('matching_cloud', LaunchConfiguration('matching_cloud_topic')),
                ('diagnostics', '/nodding_lidar/motion_cloud/diagnostics'),
            ]),
    ])
