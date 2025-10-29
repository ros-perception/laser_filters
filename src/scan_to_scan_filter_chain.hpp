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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// TF
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

#include "message_filters/subscriber.hpp"

#include "filters/filter_chain.hpp"


class ScanToScanFilterChain : public rclcpp::Node
{
protected:
  // Components for tf::MessageFilter
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  tf2_ros::Buffer buffer_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

  // Parameters
  #ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  bool lazy_subscription_;
  #endif
  std::string tf_message_filter_target_frame_;
  double tf_filter_tolerance_;
  int scan_filtered_history_depth_;

  // Diagnostic updater
  diagnostic_updater::Heartbeat heartbeat_diagnostics_;
  diagnostic_updater::Updater diagnostic_updater_;

public:
  // Constructor
  ScanToScanFilterChain(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & ns = "");

  // Destructor
  ~ScanToScanFilterChain();

  // Callback
  void callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg_in);
};
