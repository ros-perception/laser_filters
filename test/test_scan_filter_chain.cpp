/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pluginlib/class_loader.hpp>

using sensor_msgs::msg::LaserScan;

LaserScan gen_msg(rclcpp::Time stamp)
{
  LaserScan msg;

  float temp[] = {1.0, 0.1, 1.0, 1.0, 1.0, 9.0, 1.0, 1.0, 1.0, 2.3};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));

  msg.header.stamp = stamp;
  msg.header.frame_id = "laser";
  msg.angle_min = -.5;
  msg.angle_max = .5;
  msg.angle_increment = 0.1;
  msg.time_increment = 0.1;
  msg.scan_time = 0.1;
  msg.range_min = 0.5;
  msg.range_max = 1.5;
  msg.ranges = v1;
  msg.intensities = v1;

  return msg;
}

/** Verifies that two vectors of range values are the same. Allows the case
 * where corresponding values are both NaN.
 */
void expect_ranges_eq(const std::vector<float> &a, const std::vector<float> &b) {
  for( int i=0; i<10; i++) {
    if(std::isnan(a[i])) {
      EXPECT_TRUE(std::isnan(b[i]));
    }
    else {
      EXPECT_NEAR(a[i], b[i], 1e-6);
    }
  }
}

TEST(ScanToScanFilterChain, BadConfiguration)
{
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  try
  {

    rclcpp::Node::SharedPtr node =
        std::make_shared<rclcpp::Node>("bad_filter_chain");
    filter_chain_.configure(
        "",
        node->get_node_logging_interface(),
        node->get_node_parameters_interface());
  }
  catch (const pluginlib::LibraryLoadException &)
  {
    EXPECT_FALSE(false);
  }
  
  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, IntensityFilter)
{
  LaserScan msg_in, msg_out, expected_msg;
  float nanval = std::numeric_limits<float>::quiet_NaN();
  float temp[] = {1.0, nanval, 1.0, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, 2.3};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("intensity_filter_chain");
  EXPECT_TRUE(filter_chain_.configure(
      "",
      node->get_node_logging_interface(),
      node->get_node_parameters_interface()));

  msg_in = gen_msg(node->now());

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, InterpFilter)
{
  LaserScan msg_in, msg_out, expected_msg;
  float temp[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("interp_filter_chain");
  EXPECT_TRUE(filter_chain_.configure(
      "",
      node->get_node_logging_interface(),
      node->get_node_parameters_interface()));

  msg_in = gen_msg(node->now());

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  
  for( int i=0; i<10; i++){
    EXPECT_NEAR(msg_out.ranges[i],expected_msg.ranges[i],1e-6);
  }

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, ShadowFilter)
{
  LaserScan msg_in, msg_out, expected_msg;
  float nanval = std::numeric_limits<float>::quiet_NaN();
  float temp[] = {nanval, 0.1, nanval, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, nanval};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("shadow_filter_chain");
  EXPECT_TRUE(filter_chain_.configure(
      "",
      node->get_node_logging_interface(),
      node->get_node_parameters_interface()));

  msg_in = gen_msg(node->now());

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));

  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, ArrayFilter)
{
  LaserScan msg_in, msg_out, expected_msg;
  float temp[] = {1.0, 0.4, 1.0, 1.0, 1.0, 6.3333, 1.0, 1.0, 1.0, 1.8667};
  std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("array_filter_chain");
  EXPECT_TRUE(filter_chain_.configure(
      "",
      node->get_node_logging_interface(),
      node->get_node_parameters_interface()));

  msg_in = gen_msg(node->now());

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  float temp2[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<float> v2 (temp2, temp2 + sizeof(temp2) / sizeof(float));
  msg_in.ranges = v2;
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  msg_in = gen_msg(node->now());
  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));
  
  for( int i=0; i<10; i++){
    EXPECT_NEAR(msg_out.ranges[i],expected_msg.ranges[i],1e-3);
    EXPECT_NEAR(msg_out.intensities[i],msg_in.intensities[i],1e-3);
  }

  filter_chain_.clear();
}

TEST(ScanToScanFilterChain, MaskFilter)
{
  LaserScan msg_in, msg_out, expected_msg;
  const float nanval = std::numeric_limits<float>::quiet_NaN();
  const float temp[] = {1.0, nanval, 1.0, 1.0, 1.0, nanval, 1.0, 1.0, 1.0, 2.3};
  const std::vector<float> v1 (temp, temp + sizeof(temp) / sizeof(float));
  expected_msg.ranges = v1;
  filters::FilterChain<LaserScan> filter_chain_("sensor_msgs::msg::LaserScan");

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("mask_filter_chain");
  EXPECT_TRUE(filter_chain_.configure(
      "",
      node->get_node_logging_interface(),
      node->get_node_parameters_interface()));

  msg_in = gen_msg(node->now());

  EXPECT_TRUE(filter_chain_.update(msg_in, msg_out));

  expect_ranges_eq(msg_out.ranges, expected_msg.ranges);

  filter_chain_.clear();
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
