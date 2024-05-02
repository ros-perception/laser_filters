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
    config = os.path.join(get_package_share_directory("laser_filters"), "test", "test_speckle_filter.yaml")

    dist_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter_distance",
        parameters=[config],
        remappings=[("/scan_filtered", "/scan_filtered_distance")],
    )

    eucl_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter_euclidean",
        parameters=[config],
        remappings=[("/scan_filtered", "/scan_filtered_euclidean")],
    )
    return launch.LaunchDescription([dist_node, eucl_node, launch_testing.actions.ReadyToTest()])


class TestSpeckleFilter(unittest.TestCase):
    def test_speckle_filter(self):
        rclpy.init()
        node = TestFixture()
        self.assertTrue(node.wait_for_subscribers(10))
        node.start_subscribers()
        node.publish_laser_scan()

        dist_msgs_received_flag = node.dist_msg_event_object.wait(timeout=10.0)
        eucl_msgs_received_flag = node.eucl_msg_event_object.wait(timeout=10.0)
        assert dist_msgs_received_flag, "Did not receive distance msgs !"
        assert eucl_msgs_received_flag, "Did not receive distance msgs !"

        expected_scan_ranges = [1, 1, 1, 1, float("nan"), 1, 1, 1, 1, 1, 1]
        for scan_range, expected_scan_range in zip(node.msg_dist.ranges, expected_scan_ranges):
            if math.isnan(expected_scan_range) or math.isnan(scan_range):
                self.assertEqual(math.isnan(expected_scan_range), math.isnan(scan_range))
            else:
                self.assertEqual(scan_range, expected_scan_range)

        # NOTE: This is the actual behavior you would get in ROS1, but there was a bug because of which it would never go into euclidean mode
        expected_scan_ranges = [
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
        ]
        for scan_range, expected_scan_range in zip(node.msg_euclid.ranges, expected_scan_ranges):
            if math.isnan(expected_scan_range) or math.isnan(scan_range):
                self.assertEqual(math.isnan(expected_scan_range), math.isnan(scan_range))
            else:
                self.assertEqual(scan_range, expected_scan_range)


class TestFixture(Node):
    def __init__(self):
        super().__init__("test_speckle_filter_distance")
        self.dist_msg_event_object = Event()
        self.eucl_msg_event_object = Event()
        self.publisher = self.create_publisher(LaserScan, "scan", 10)

    def wait_for_subscribers(self, timeout):
        timer_period = 0.1
        t = 0.0
        rate = self.create_rate(1 / timer_period, self.get_clock())
        while rclpy.ok() and t < timeout:
            rclpy.spin_once(self)
            if self.publisher.get_subscription_count() >= 2:
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
        msg.ranges = [1.0, 1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        msg.range_max = 100.0
        self.publisher.publish(msg)

    def start_subscribers(self):
        self.distance_subscriber = self.create_subscription(LaserScan, "scan_filtered_distance", self.dist_cb, 10)

        self.euclidean_subscriber = self.create_subscription(LaserScan, "scan_filtered_euclidean", self.euclid_cb, 10)

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def dist_cb(self, msg):
        self.msg_dist = msg
        self.dist_msg_event_object.set()
        print("received dist")
        print(self.msg_dist)

    def euclid_cb(self, msg):
        self.msg_euclid = msg
        self.eucl_msg_event_object.set()
        print("received euclid")
        print(self.msg_euclid)
