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


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <iostream>
#include <memory>
// TF
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "filters/filter_chain.hpp"

typedef tf2::TransformException TransformException;
typedef tf2_ros::TransformListener TransformListener;

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  rclcpp::Node::SharedPtr nh_;

  // Components for tf::MessageFilter
  TransformListener tf_;
  tf2_ros::Buffer buffer_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> tf_filter_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

  // Timer for displaying deprecation warnings
  rclcpp::TimerBase::SharedPtr deprecation_timer_;

private:
  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");
  void foo(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
  }

public:
  // Constructor
  explicit GenericLaserScanFilterNode(rclcpp::Node::SharedPtr nh)
  : nh_(nh),
    scan_sub_(nh_, "scan"),
    buffer_(nh->get_clock()),
    tf_(buffer_),
    tf_filter_(scan_sub_, buffer_, "base_link", 50, 0),
    filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // Configure filter chain
    filter_chain_.configure("", nh_);

    // Setup tf::MessageFilter for input
    tf_filter_.registerCallback(std::bind(&GenericLaserScanFilterNode::callback, this,
      std::placeholders::_1));
    rclcpp::Duration tolerance = rclcpp::Duration(0.03);
    tolerance.nanoseconds();
    tf_filter_.setTolerance(tolerance);

    // Advertise output
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("output", 1000);

    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> standard_callback =
      std::bind(&GenericLaserScanFilterNode::foo, this, std::placeholders::_1);
    nh_->create_subscription<sensor_msgs::msg::LaserScan>("scan", standard_callback,
      rmw_qos_profile_default);

    deprecation_timer_ =
      nh_->create_wall_timer(std::chrono::milliseconds(5000),
        std::bind(&GenericLaserScanFilterNode::deprecation_warn, this));
  }

  void deprecation_warn()
  {
    RCLCPP_WARN(laser_filters_logger,
      "'generic_laser_filter_node' has been deprecated. Please use 'scan_to_scan_filter_chain'.");
  }

  // Callback
  void callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg_in)
  {
    // Run the filter chain
    filter_chain_.update(*msg_in, msg_);

    // Publish the output
    output_pub_->publish(msg_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_filter_node");
  GenericLaserScanFilterNode t(nh);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }

  return 0;
}
