# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import launch_testing.actions
import math
import pytest
import os.path
import rclpy
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node as RosNode
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from typing import Optional

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        # Launch the filter node using the example yaml
        RosNode(
            package='sensor_filters',
            executable='laser_scan_filter_chain',
            name='laser_filter_test',
            parameters=[os.path.join(os.path.dirname(__file__), '..', 'examples', 'filter.yaml')],
            remappings=[('/input', '/test_scan'), ('/output', '/test_scan_filtered')]
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestLaserFilter(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_node')
        self.pub = self.node.create_publisher(LaserScan, '/test_scan', 10)
        self.sub = self.node.create_subscription(LaserScan, '/test_scan_filtered', self.callback, 10)
        self.received_msg: Optional[LaserScan] = None

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # TODO Humble: remove the first if when Humble is not needed
            if hasattr(self.pub, 'get_subscription_count') and hasattr(self.sub, 'get_publisher_count'):
                if self.pub.get_subscription_count() > 0 and self.sub.get_publisher_count() > 0:
                    break

    def tearDown(self):
        self.node.destroy_node()

    def callback(self, msg):
        self.received_msg = msg

    def test_timestamp_offset(self):
        msg = LaserScan()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        self.pub.publish(msg)
        
        # Wait for output
        start_time = time.time()
        while self.received_msg is None and (time.time() - start_time) < 60:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.received_msg)
        self.assertEqual(self.received_msg.header.stamp.sec, 9)
        self.assertEqual(self.received_msg.header.stamp.nanosec, 975000000)

        # The intensity filter filters out rays with intensities outside <8'000, 100'000>
        # It is configured to set ranges of invalid rays to NaN and to not touch the corresponding intensities

        self.received_msg = None

        msg.range_min = 0.1
        msg.range_max = 50.0
        msg.ranges = [0.0, 1.0]
        msg.intensities = [0.0, 10000.0]
        self.pub.publish(msg)

        # Wait for output
        start_time = time.time()
        while self.received_msg is None and (time.time() - start_time) < 60:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.received_msg)
        self.assertEqual(self.received_msg.header.stamp.sec, 9)
        self.assertEqual(self.received_msg.header.stamp.nanosec, 975000000)
        self.assertEqual(len(self.received_msg.ranges), 2)
        self.assertEqual(len(self.received_msg.intensities), 2)
        self.assertTrue(math.isnan(self.received_msg.ranges[0]))
        self.assertEqual(self.received_msg.ranges[1], 1.0)
        self.assertSequenceEqual(self.received_msg.intensities, msg.intensities)
