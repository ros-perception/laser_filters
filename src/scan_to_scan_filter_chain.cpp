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
#include <sensor_msgs/msg/Laser_Scan.hpp>

// TF
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

#include "message_filters/subscriber.h"

// TODO: fix this
#define NO_TIMER

#include "filters/filter_chain.h"

class ScanToScanFilterChain
{
protected:
  // Our NodeHandle
  rclcpp::Node::SharedPtr nh_;

  // Components for tf::MessageFilter
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  tf2_ros::Buffer buffer_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;
  double tf_filter_tolerance_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

  // Deprecation helpers
#ifndef NO_TIMER
  ros::Timer deprecation_timer_;
#endif // !NO_TIMER
  bool  using_filter_chain_deprecated_;

public:
  // Constructor
  ScanToScanFilterChain(rclcpp::Node::SharedPtr node) :
    nh_(node),
    scan_sub_(nh_, "scan", 50),
    tf_(NULL),
    tf_filter_(NULL),
    filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // Configure filter chain
    
    rclcpp::parameter::ParameterVariant variant;
    using_filter_chain_deprecated_ = !nh_->get_parameter("filter_chain", variant);

    if (using_filter_chain_deprecated_)
      filter_chain_.configure("filter_chain", nh_);
    else
      filter_chain_.configure("scan_filter_chain", nh_);
    
    std::string tf_message_filter_target_frame;

    if (nh_->get_parameter("tf_message_filter_target_frame", variant))
    {
      nh_->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame);

      nh_->get_parameter_or("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_.reset(new tf2_ros::TransformListener(buffer_));
      tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(scan_sub_, buffer_, "", 50));
      tf_filter_->setTargetFrame(tf_message_filter_target_frame);
      tf_filter_->setTolerance(tf2::Duration(ros::Duration(tf_filter_tolerance_).toNSec()));

      // Setup tf::MessageFilter generates callback
      tf_filter_->registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }
    else 
    {
      // Pass through if no tf_message_filter_target_frame
      scan_sub_.registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }
    
    // Advertise output
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 1000);

#ifndef NO_TIMER
    // Set up deprecation printout
    deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&ScanToScanFilterChain::deprecation_warn, this, _1));
#endif // !NO_TIMER
  }

  // Destructor
  ~ScanToScanFilterChain()
  {
    if (tf_filter_)
      tf_filter_.reset();
    if (tf_)
      tf_.reset();
  }
  
#ifndef NO_TIMER
  // Deprecation warning callback
  void deprecation_warn(const ros::TimerEvent& e)
  {
    if (using_filter_chain_deprecated_)
      ROS_WARN("Use of '~filter_chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~scan_filter_chain'.");
  }
#endif // !NO_TIMER

  // Callback
  void callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg_in)
  {
    // Run the filter chain
    if (filter_chain_.update(*msg_in, msg_))
    {
      //only publish result if filter succeeded
      output_pub_->publish(msg_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::Time::init();
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_to_scan_filter_chain");
  ScanToScanFilterChain t(nh);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {

    rclcpp::spin_some(nh);
    loop_rate.sleep();

  }

  return 0;
}
