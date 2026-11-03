"""独立したROS Domainで、計測時刻の座標変換・欠測・時刻逆行を検証する。"""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import unittest

# 実機や実行中のシミュレーションには接続しない。
os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 80)

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

EXECUTABLE = Path(sys.argv.pop(1)).resolve()


def stamp(value):
    """秒単位の試験時刻をROS Timeメッセージへ変換する。"""
    from builtin_interfaces.msg import Time
    nanos = round(value * 1e9)
    return Time(sec=nanos // 1000000000, nanosec=nanos % 1000000000)


class MotionCloudRosTest(unittest.TestCase):
    """静止物の重なりと、最新TFの代用禁止を実プロセスで確認する。"""

    def test_measurement_frames_and_faults(self):
        """旋回中の静止点の一致、履歴不足・TF欠測・時計逆行時の出力を確認する。"""
        rclpy.init()
        node = rclpy.create_node('motion_cloud_test')
        process = subprocess.Popen([
            str(EXECUTABLE), '--ros-args', '--params-file',
            str(Path(__file__).parents[1] / 'config/motion_cloud.yaml'),
            '-p', 'use_sim_time:=true',
            '-p', 'min_points:=1', '-p', 'voxel_size_m:=0.0',
            '-p', 'max_points:=50', '-p', 'max_output_rate_hz:=10.0',
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        outputs, diagnostics = [], []
        clock = node.create_publisher(Clock, '/clock', 10)
        tf = node.create_publisher(TFMessage, '/tf', 20)
        odom = node.create_publisher(Odometry, '/odometry', 20)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud', qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/matching_cloud', outputs.append, 10)
        node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10)

        def pump(duration=0.12):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertIsNone(process.poll(), 'Assembler exited unexpectedly')

        def set_clock(t):
            for _ in range(8):
                clock.publish(Clock(clock=stamp(t)))
                pump(0.05)

        def transform(t):
            delta = t - 100.0
            yaw = 0.4 * delta
            msg = TransformStamped()
            msg.header.frame_id = 'odom'
            msg.header.stamp = stamp(t)
            msg.child_frame_id = 'base_link'
            msg.transform.translation.x = 0.1 * delta
            msg.transform.rotation.z = math.sin(yaw / 2)
            msg.transform.rotation.w = math.cos(yaw / 2)
            return msg

        def geometry(t, motion=True):
            tf.publish(TFMessage(transforms=[transform(t - 0.05), transform(t),
                                            transform(t + 0.05)]))
            if motion:
                for sample in (t - 0.05, t, t + 0.05):
                    msg = Odometry()
                    msg.header.frame_id = 'odom'
                    msg.header.stamp = stamp(sample)
                    msg.child_frame_id = 'base_link'
                    msg.twist.twist.angular.z = 0.4
                    odom.publish(msg)
            pump()

        def cloud(t, count=1):
            # 世界座標で固定された点(2, 1, 0)を、計測時刻の車体座標で入力する。
            delta = t - 100.0
            yaw = 0.4 * delta
            x = math.cos(yaw) * (2 - 0.1 * delta) + math.sin(yaw)
            y = -math.sin(yaw) * (2 - 0.1 * delta) + math.cos(yaw)
            msg = PointCloud2()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = stamp(t)
            msg.height, msg.width = 1, count
            msg.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
            msg.point_step, msg.row_step = 12, count * 12
            msg.data = struct.pack('<fff', x, y, 0.0) * count
            cloud_pub.publish(msg)
            return x, y

        try:
            deadline = time.monotonic() + 8
            while (cloud_pub.get_subscription_count() == 0 or
                   odom.get_subscription_count() == 0 or tf.get_subscription_count() == 0 or
                   clock.get_subscription_count() == 0 or
                   node.count_publishers('/matching_cloud') == 0) and time.monotonic() < deadline:
                pump()
            self.assertGreater(cloud_pub.get_subscription_count(), 0)
            set_clock(100.0)
            geometry(100.0)
            cloud(100.0)
            pump(1.3)
            self.assertTrue(outputs, str(diagnostics))
            self.assertEqual(outputs[-1].width, 1)
            set_clock(100.1)
            geometry(100.1)
            expected = cloud(100.1)
            pump(0.3)
            result = outputs[-1]
            self.assertEqual(result.header.stamp, stamp(100.1))
            self.assertEqual(result.header.frame_id, 'base_link')
            self.assertEqual(result.width, 2)
            for offset in (0, result.point_step):
                x, y, _ = struct.unpack_from('<fff', bytes(result.data), offset)
                self.assertAlmostEqual(x, expected[0], places=5)
                self.assertAlmostEqual(y, expected[1], places=5)

            # オドメトリの履歴がないときは古い点を足さず、単発点群へ退避する。
            set_clock(101.0)
            geometry(101.0, motion=False)
            cloud(101.0)
            pump(0.6)
            self.assertEqual(outputs[-1].header.stamp, stamp(101.0))
            self.assertEqual(outputs[-1].width, 1)
            pump(1.1)
            self.assertIn('single_scan_without_motion_history', diagnostics[-1].status[0].message)

            # TFの範囲外で、最新TFを代用して出力しない。
            set_clock(101.2)
            previous = len(outputs)
            cloud(101.2)
            pump(0.6)
            self.assertEqual(len(outputs), previous)
            cloud(101.2)  # 重複stampも無視する。
            pump()
            self.assertEqual(len(outputs), previous)
            geometry(101.3)
            cloud(101.3, count=51)  # 入力の点数上限。
            pump()
            self.assertEqual(len(outputs), previous)

            # 時刻を戻した後は、前の走行の点群を再利用しない。
            set_clock(90.0)
            geometry(90.0)
            cloud(90.0)
            pump(0.4)
            self.assertEqual(outputs[-1].header.stamp, stamp(90.0))
            self.assertEqual(outputs[-1].width, 1)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                log, _ = process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                log, _ = process.communicate()
            print(log.decode(errors='replace'), file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
