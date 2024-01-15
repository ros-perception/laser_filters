#!/usr/bin/env python
#
# Copyright (c) 2020, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from threading import Thread, Event
import math
import unittest
import launch
import launch.actions
import launch.substitutions
import launch_testing
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.join(get_package_share_directory("laser_filters"), "test", "test_polygon_filter.yaml")

    node = launch_ros.actions.Node(package="laser_filters", executable="scan_to_scan_filter_chain", parameters=[config])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestPolygonFilter(unittest.TestCase):
    def test_polygon_filter(self):
        rclpy.init()
        node = TestFixture()
        self.assertTrue(node.wait_for_subscriber(10))
        node.start_subscriber()
        node.publish_laser_scan()

        msgs_received_flag = node._msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, "Did not receive msgs !"
        expected_scan_ranges = [1.0, 1.0, 1.0, 1.0, float("nan"), float("nan"), float("nan"), 1, 1, 1, 1]
        for scan_range, expected_scan_range in zip(node._received_message.ranges, expected_scan_ranges):
            if math.isnan(expected_scan_range) or math.isnan(scan_range):
                self.assertEqual(
                    math.isnan(expected_scan_range),
                    math.isnan(scan_range),
                    "failed %f and %f" % (expected_scan_range, scan_range),
                )
                pass
            else:
                self.assertEqual(scan_range, expected_scan_range)
        node.destroy_node()
        rclpy.shutdown()


class TestFixture(Node):
    def __init__(self):
        super().__init__("test_polygon_filter")
        self._msg_event_object = Event()
        self._publisher = self.create_publisher(LaserScan, "scan", 10)

    def wait_for_subscriber(self, timeout):
        timer_period = 0.1
        t = 0.0
        rate = self.create_rate(1 / timer_period, self.get_clock())
        while rclpy.ok() and t < timeout:
            rclpy.spin_once(self)
            if self._publisher.get_subscription_count() > 0:
                return True
            rate.sleep()
            t += timer_period
        return False

    def publish_laser_scan(self):
        num_beams = 11
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / (num_beams - 1)
        msg.ranges = [1.0] * num_beams
        msg.range_max = 100.0
        self._publisher.publish(msg)

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(LaserScan, "scan_filtered", self.callback, 10)

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def callback(self, message):
        self._msg_event_object.set()
        self._received_message = message
