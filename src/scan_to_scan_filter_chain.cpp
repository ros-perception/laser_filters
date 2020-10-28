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

// TF
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

#include "message_filters/subscriber.h"

#include "filters/filter_chain.hpp"

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

public:
  // Constructor
  ScanToScanFilterChain(rclcpp::Node::SharedPtr node)
      : nh_(node),
        tf_(NULL),
        buffer_(nh_->get_clock()),
        scan_sub_(nh_, "scan", rmw_qos_profile_sensor_data),
        tf_filter_(NULL),
        filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // Configure filter chain
    filter_chain_.configure("", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());

    std::string tf_message_filter_target_frame;
    if (nh_->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame))
    {

      nh_->get_parameter_or("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_.reset(new tf2_ros::TransformListener(buffer_));
      tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(scan_sub_, buffer_, "", 50, nh_));
      tf_filter_->setTargetFrame(tf_message_filter_target_frame);
      tf_filter_->setTolerance(std::chrono::duration<double>(tf_filter_tolerance_));

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
  }

  // Destructor
  ~ScanToScanFilterChain()
  {
    if (tf_filter_)
      tf_filter_.reset();
    if (tf_)
      tf_.reset();
  }

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
