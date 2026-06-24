# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import launch_testing.actions
import pytest
import os.path
import rclpy
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node as RosNode
from point_cloud_interfaces.msg import CompressedPointCloud2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from typing import Optional

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        # Launch the image filter node using the example yaml
        RosNode(
            package='sensor_filters',
            executable='pointcloud2_filter_chain',
            name='pointcloud2_filter_test',
            parameters=[os.path.join(os.path.dirname(__file__), '..', 'examples', 'pointcloud2_filter.yaml')],
            remappings=[('/input', '/test_cloud'), ('/output', '/test_cloud_filtered')]
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestPointCloud2Filter(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_node')
        self.pub = self.node.create_publisher(CompressedPointCloud2, '/test_cloud/draco', 10)
        self.sub_raw = self.node.create_subscription(PointCloud2, '/test_cloud_filtered', self.callback_raw, 10)
        self.sub_compressed = self.node.create_subscription(
            CompressedPointCloud2, '/test_cloud_filtered/draco', self.callback_compressed, 10)
        self.received_raw_msg: Optional[PointCloud2] = None
        self.received_compressed_msg: Optional[CompressedPointCloud2] = None

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # TODO Humble: remove the first if when Humble is not needed
            if hasattr(self.pub, 'get_subscription_count') and hasattr(self.sub_raw, 'get_publisher_count'):
                if self.pub.get_subscription_count() > 0 and self.sub_raw.get_publisher_count() > 0 and \
                        self.sub_compressed.get_publisher_count() > 0:
                    break

    def tearDown(self):
        self.node.destroy_node()

    def callback_raw(self, msg: PointCloud2):
        self.received_raw_msg = msg

    def callback_compressed(self, msg: CompressedPointCloud2):
        self.received_compressed_msg = msg

    def test_timestamp_offset_compressed(self):
        # Construct a CompressedPointCloud2 message
        msg = CompressedPointCloud2()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "map"
        msg.format = "draco"
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.width = 1
        msg.height = 1
        msg.point_step = 12
        msg.row_step = 12
        msg.is_bigendian = False
        msg.is_dense = True
        # This is the compressed transport representation of a cloud with a single zero XYZ point
        msg.compressed_data = [
            68, 82, 65, 67, 79, 2, 3, 0, 0, 0, 128, 0, 1, 11, 100, 101, 100, 117, 112, 108, 105, 99, 97, 116, 101, 4, 1,
            0, 0, 0, 0, 1, 0, 0, 0, 1, 3, 0, 9, 1, 0, 0, 0, 9, 1, 0, 1, 0, 9, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0
        ]
        
        self.pub.publish(msg)
    
        # Wait for output
        start_time = time.time()
        while (self.received_raw_msg is None or self.received_compressed_msg is None) and \
                (time.time() - start_time) < 10:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.received_raw_msg)
        self.assertIsNotNone(self.received_compressed_msg)

        self.assertEqual(self.received_raw_msg.header.stamp.sec, 9)
        self.assertEqual(self.received_raw_msg.header.stamp.nanosec, 975000000)
        self.assertEqual(self.received_raw_msg.header.frame_id, "map")
        self.assertEqual(self.received_raw_msg.width, 1)
        self.assertEqual(self.received_raw_msg.height, 1)
        self.assertEqual(self.received_raw_msg.point_step, 12)
        self.assertEqual(self.received_raw_msg.row_step, 12)
        self.assertEqual(self.received_raw_msg.is_dense, True)
        self.assertEqual(self.received_raw_msg.is_bigendian, False)
        self.assertSequenceEqual(self.received_raw_msg.fields, msg.fields)
        self.assertSequenceEqual(self.received_raw_msg.data, [0] * 12)

        self.assertEqual(self.received_compressed_msg.header.stamp.sec, 9)
        self.assertEqual(self.received_compressed_msg.header.stamp.nanosec, 975000000)
        self.assertEqual(self.received_compressed_msg.header.frame_id, "map")
        self.assertEqual(self.received_compressed_msg.width, 1)
        self.assertEqual(self.received_compressed_msg.height, 1)
        self.assertEqual(self.received_compressed_msg.point_step, 12)
        self.assertEqual(self.received_compressed_msg.row_step, 12)
        self.assertEqual(self.received_compressed_msg.is_dense, True)
        self.assertEqual(self.received_compressed_msg.is_bigendian, False)
        self.assertSequenceEqual(self.received_compressed_msg.fields, msg.fields)
        self.assertSequenceEqual(self.received_compressed_msg.compressed_data, msg.compressed_data)
