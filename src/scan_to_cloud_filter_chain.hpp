/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
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
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
\author Radu Bogdan Rusu <rusu@cs.tum.edu>


 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// TF
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"

#include "message_filters/subscriber.h"

// Laser projection
#include <laser_geometry/laser_geometry.hpp>

//Filters
#include "filters/filter_chain.hpp"

namespace laser_filters
{
class ScanToCloudFilterChain : public rclcpp::Node
{
public:
  // ROS related
  laser_geometry::LaserProjection projector_; // Used to project laser scans

  double laser_max_range_; // Used in laser scan projection
  int window_;

  bool high_fidelity_;                    // High fidelity (interpolating time across scan)
  std::string target_frame_;                   // Target frame for high fidelity result
  std::string scan_topic_, cloud_topic_;

  // TF
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> filter_;

  double tf_tolerance_;
  filters::FilterChain<sensor_msgs::msg::PointCloud2> cloud_filter_chain_;
  filters::FilterChain<sensor_msgs::msg::LaserScan> scan_filter_chain_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  bool incident_angle_correction_;

  ScanToCloudFilterChain(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & ns = "");

  void
  scanCallback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & scan_msg);
};
}  // namespace laser_filters
