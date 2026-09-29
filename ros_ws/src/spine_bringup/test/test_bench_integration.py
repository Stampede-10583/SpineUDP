"""End-to-end: fake RIO <-> companion + synthetic pose node, over real UDP on loopback.

Checks the guarantees the RIO depends on:
  - every Stream 2 datagram is exactly 40 bytes;
  - every Stream 2 timestamp is bit-for-bit the timestamp of a trigger the RIO sent;
  - Stream 2 timestamps strictly increase, across a robot-program restart too;
and that the companion's counters see the injected faults (no-fix, stale, out-of-order, a lost
trigger burst, a junk datagram, a restart).
"""

import os
import struct
import sys
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from spine_interfaces.msg import LinkStats

sys.path.insert(0, os.path.dirname(__file__))
from fake_rio import FakeRio  # noqa: E402

# Off the default ports, so a running bench or RIO sim can't interfere.
ODOM_PORT = 25801
POSE_PORT = 25802
TRIG_PORT = 25803


@pytest.mark.launch_test
def generate_test_description():
    params = os.path.join(get_package_share_directory('spine_bringup'), 'config', 'spine.yaml')
    ipc = [{'use_intra_process_comms': True}]
    container = ComposableNodeContainer(
        name='spine_test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spine_link', plugin='spine::SpineCompanionNode',
                name='spine_companion',
                parameters=[params, {
                    'bind_address': '127.0.0.1', 'rio_address': '127.0.0.1',
                    'odometry_port': ODOM_PORT, 'trigger_port': TRIG_PORT,
                    'rio_pose_port': POSE_PORT, 'stats_period_s': 0.25}],
                extra_arguments=ipc),
            ComposableNode(
                package='spine_link', plugin='spine::SyntheticPoseNode',
                name='spine_synthetic_pose',
                parameters=[params, {
                    'latency_mean_s': 0.04, 'latency_jitter_s': 0.03,
                    'drop_probability': 0.05, 'invalid_probability': 0.1,
                    'stale_probability': 0.05, 'stale_delay_s': 2.0, 'seed': 42}],
                extra_arguments=ipc),
        ],
        output='screen')
    return launch.LaunchDescription([container, launch_testing.actions.ReadyToTest()])


class TestBench(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('spine_bench_test')
        cls.stats = []
        cls.node.create_subscription(LinkStats, 'spine/stats', cls.stats.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_link_end_to_end(self):
        rio = FakeRio('127.0.0.1', ODOM_PORT, TRIG_PORT, POSE_PORT)
        try:
            self.spin_for(1.5)  # let the container come up
            rio.run(4.0)                   # run 1, with network faults
            rio.run(3.0, faults=False)     # robot-program restart: seqs from 0, clock continues
            self.spin_for(3.0)             # stale poses flush, final stats arrive
        finally:
            rio.close()

        poses = list(rio.poses)
        self.assertGreater(len(poses), 100, 'expected a steady stream of poses')

        for d in poses:
            self.assertEqual(len(d), 40)

        stamps = []
        for d in poses:
            raw = d[8:16]
            self.assertIn(raw, rio.trigger_ts_bytes,
                          'Stream 2 timestamp is not bit-identical to any trigger sent')
            stamps.append(struct.unpack('<d', raw)[0])
        for older, newer in zip(stamps, stamps[1:]):
            self.assertGreater(newer, older, 'Stream 2 timestamps must strictly increase')

        self.assertTrue(self.stats, 'no spine/stats received')
        s = self.stats[-1]
        print(f'[bench] poses={len(poses)} sent={s.poses_sent} noFix={s.withheld_no_fix} '
              f'stale={s.withheld_stale} outOfOrder={s.withheld_out_of_order} '
              f'restarts={s.rio_restarts} missed={s.triggers_missed} '
              f'decodeErrors={s.decode_errors} odom={s.odom_recvd} trig={s.trig_recvd}')
        self.assertEqual(s.poses_sent, len(poses))
        self.assertGreater(s.withheld_no_fix, 0)
        self.assertGreater(s.withheld_stale, 0)
        self.assertGreater(s.withheld_out_of_order, 0, 'latency jitter should reorder results')
        self.assertEqual(s.withheld_bad_timestamp, 0)
        self.assertEqual(s.rio_restarts, 1)
        self.assertEqual(s.triggers_missed, rio.lost_triggers)
        self.assertEqual(s.decode_errors, rio.junk_sent)
        self.assertEqual(s.send_errors, 0)
