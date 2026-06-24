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
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from typing import Optional

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        # Launch the image filter node using the example yaml
        RosNode(
            package='sensor_filters',
            executable='image_filter_chain',
            name='image_filter_test',
            parameters=[os.path.join(os.path.dirname(__file__), '..', 'examples', 'image_filter.yaml')],
            remappings=[('/input', '/test_image'), ('/output', '/test_image_filtered')]
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestImageFilter(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_node')
        self.pub = self.node.create_publisher(CompressedImage, '/test_image/compressed', 10)
        self.sub_raw = self.node.create_subscription(Image, '/test_image_filtered', self.callback_raw, 10)
        self.sub_compressed = self.node.create_subscription(
            CompressedImage, '/test_image_filtered/compressed', self.callback_compressed, 10)
        self.received_raw_msg: Optional[Image] = None
        self.received_compressed_msg: Optional[CompressedImage] = None

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # TODO Humble: remove the first if when Humble is not needed
            if hasattr(self.pub, 'get_subscription_count') and hasattr(self.sub_raw, 'get_publisher_count'):
                if self.pub.get_subscription_count() > 0 and self.sub_raw.get_publisher_count() > 0 and \
                        self.sub_compressed.get_publisher_count() > 0:
                    break

    def tearDown(self):
        self.node.destroy_node()

    def callback_raw(self, msg: Image):
        self.received_raw_msg = msg

    def callback_compressed(self, msg: CompressedImage):
        self.received_compressed_msg = msg

    def test_timestamp_offset_compressed(self):
        # Construct a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "camera"
        msg.format = "mono8; jpeg compressed mono8"
        # This is the compressed transport representation of an image with one mono8 pixel with value 0
        msg.data = [
            255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67, 0, 2, 1,
            1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6, 6, 6, 5, 6, 6, 6,
            7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11, 8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12, 11, 10, 12, 9, 10, 10, 10,
            255, 192, 0, 11, 8, 0, 1, 0, 1, 1, 1, 17, 0, 255, 196, 0, 31, 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 255, 196, 0, 181, 16, 0, 2, 1, 3, 3, 2, 4, 3, 5,
            5, 4, 4, 0, 0, 1, 125, 1, 2, 3, 0, 4, 17, 5, 18, 33, 49, 65, 6, 19, 81, 97, 7, 34, 113, 20, 50, 129,
            145, 161, 8, 35, 66, 177, 193, 21, 82, 209, 240, 36, 51, 98, 114, 130, 9, 10, 22, 23, 24, 25, 26,
            37, 38, 39, 40, 41, 42, 52, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84, 85, 86,
            87, 88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120, 121, 122, 131,
            132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162, 163, 164, 165,
            166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194, 195, 196, 197, 198, 199,
            200, 201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 225, 226, 227, 228, 229, 230, 231, 232,
            233, 234, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 255, 218, 0, 8, 1, 1, 0, 0, 63, 0, 254,
            127, 235, 255, 217
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
        self.assertEqual(self.received_raw_msg.header.frame_id, "camera")
        self.assertEqual(self.received_raw_msg.width, 1)
        self.assertEqual(self.received_raw_msg.height, 1)
        self.assertEqual(self.received_raw_msg.step, 1)
        self.assertEqual(self.received_raw_msg.encoding, "mono8")
        self.assertSequenceEqual(self.received_raw_msg.data, [0.0])

        self.assertEqual(self.received_compressed_msg.header.stamp.sec, 9)
        self.assertEqual(self.received_compressed_msg.header.stamp.nanosec, 975000000)
        self.assertEqual(self.received_compressed_msg.header.frame_id, "camera")
        self.assertEqual(self.received_compressed_msg.format, msg.format)
        self.assertSequenceEqual(self.received_compressed_msg.data, msg.data)
