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
#include <float.h>
#include <string>
#include <memory>

// TF
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

// Laser projection
#include "laser_geometry/laser_geometry.hpp"

// Filters
#include "filters/filter_chain.hpp"


/** @b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class ScanToCloudFilterChain
{
private:
  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");

public:
  // ROS related
  laser_geometry::LaserProjection projector_;  // Used to project laser scans

  double laser_max_range_;           // Used in laser scan projection
  int window_;

  bool high_fidelity_;                    // High fidelity (interpolating time across scan)
  std::string target_frame_;                   // Target frame for high fidelity result
  std::string scan_topic_, cloud_topic_;

  rclcpp::Node::SharedPtr nh;

  // TF
  tf2_ros::TransformListener tf_;
  tf2_ros::Buffer buffer_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> filter_;

  double tf_tolerance_;
  filters::FilterChain<sensor_msgs::msg::PointCloud2> cloud_filter_chain_;
  filters::FilterChain<sensor_msgs::msg::LaserScan> scan_filter_chain_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // Timer for displaying deprecation warnings
  rclcpp::TimerBase::SharedPtr deprecation_timer_;

  bool using_scan_topic_deprecated_;
  bool using_cloud_topic_deprecated_;
  bool using_default_target_frame_deprecated_;
  bool using_laser_max_range_deprecated_;
  bool using_filter_window_deprecated_;
  bool using_scan_filters_deprecated_;
  bool using_cloud_filters_deprecated_;
  bool using_scan_filters_wrong_deprecated_;
  bool using_cloud_filters_wrong_deprecated_;
  bool incident_angle_correction_;

  explicit ScanToCloudFilterChain(rclcpp::Node::SharedPtr node)
  : nh(node),
    laser_max_range_(DBL_MAX),
    sub_(nh, "scan"),
    buffer_(node->get_clock()),
    tf_(buffer_),
    filter_(sub_, buffer_, "", 50, 0),
    cloud_filter_chain_("sensor_msgs::msg::PointCloud2"),
    scan_filter_chain_("sensor_msgs::msg::LaserScan")
  {
    nh->get_parameter_or("high_fidelity", high_fidelity_, false);
    nh->get_parameter_or("notifier_tolerance", tf_tolerance_, 0.03);
    nh->get_parameter_or("target_frame", target_frame_, std::string("base_link"));

    rclcpp::Parameter variant;

    // DEPRECATED with default value
    using_default_target_frame_deprecated_ = !nh->get_parameter("target_frame", variant);

    // DEPRECATED
    using_scan_topic_deprecated_ = nh->get_parameter("scan_topic", variant);
    using_cloud_topic_deprecated_ = nh->get_parameter("cloud_topic", variant);
    using_laser_max_range_deprecated_ = nh->get_parameter("laser_max_range", variant);
    using_filter_window_deprecated_ = nh->get_parameter("filter_window", variant);
    using_cloud_filters_deprecated_ = nh->get_parameter("cloud_filters/filter_chain", variant);
    using_scan_filters_deprecated_ = nh->get_parameter("scan_filters/filter_chain", variant);
    using_cloud_filters_wrong_deprecated_ = nh->get_parameter("cloud_filters/cloud_filter_chain",
        variant);
    using_scan_filters_wrong_deprecated_ = nh->get_parameter("scan_filters/scan_filter_chain",
        variant);


    nh->get_parameter_or("filter_window", window_, 2);
    nh->get_parameter_or("laser_max_range", laser_max_range_, DBL_MAX);
    nh->get_parameter_or("scan_topic", scan_topic_, std::string("tilt_scan"));
    nh->get_parameter_or("cloud_topic", cloud_topic_, std::string("tilt_laser_cloud_filtered"));
    nh->get_parameter_or("incident_angle_correction", incident_angle_correction_, true);

    filter_.setTargetFrame(target_frame_);
    filter_.registerCallback(std::bind(&ScanToCloudFilterChain::scanCallback, this,
      std::placeholders::_1));
    rclcpp::Duration tolerance = rclcpp::Duration(tf_tolerance_);
    tolerance.nanoseconds();
    filter_.setTolerance(tolerance);

    if (using_scan_topic_deprecated_) {
      sub_.subscribe(nh, scan_topic_);
    } else {
      sub_.subscribe(nh, "scan");
    }

    filter_.connectInput(sub_);

    if (using_cloud_topic_deprecated_) {
      cloud_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
    } else {
      cloud_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 10);
    }

    // std::string cloud_filter_xml;

    if (using_cloud_filters_deprecated_) {
      cloud_filter_chain_.configure("cloud_filters/filter_chain", nh);
    } else if (using_cloud_filters_wrong_deprecated_) {
      cloud_filter_chain_.configure("cloud_filters/cloud_filter_chain", nh);
    } else {
      cloud_filter_chain_.configure("cloud_filter_chain", nh);
    }

    if (using_scan_filters_deprecated_) {
      scan_filter_chain_.configure("scan_filter/filter_chain", nh);
    } else if (using_scan_filters_wrong_deprecated_) {
      scan_filter_chain_.configure("scan_filters/scan_filter_chain", nh);
    } else {
      scan_filter_chain_.configure("scan_filter_chain", nh);
    }

    deprecation_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(5000),
        std::bind(&ScanToCloudFilterChain::deprecation_warn, this));
  }

  // We use a deprecation warning on a timer to avoid warnings getting lost in the noise
  void deprecation_warn()
  {
    if (using_scan_topic_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "Use of '~scan_topic' parameter in scan_to_cloud_filter_chain has been deprecated.");
    }

    if (using_cloud_topic_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "Use of '~cloud_topic' parameter in scan_to_cloud_filter_chain has been deprecated.");
    }

    if (using_laser_max_range_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "Use of '~laser_max_range' parameter in scan_to_cloud_filter_chain has been deprecated.");
    }

    if (using_filter_window_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "Use of '~filter_window' parameter in scan_to_cloud_filter_chain has been deprecated.");
    }

    if (using_default_target_frame_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "Using '~target_frame' parameter in scan_to_cloud_filter_chain has been deprecated.");
      RCLCPP_WARN(laser_filters_logger,
        "Default currently set to 'base_link' please set explicitly as appropriate.");
    }

    if (using_cloud_filters_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "'~cloud_filters/filter_chain' param in scan_to_cloud_filter_chain has been deprecated");
      RCLCPP_WARN(laser_filters_logger,
        "Replace with '~cloud_filter_chain'");
    }

    if (using_scan_filters_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "'~scan_filters/filter_chain' param in scan_to_cloud_filter_chain has been deprecated.");
      RCLCPP_WARN(laser_filters_logger,
        "Replace with '~scan_filter_chain'");
    }

    if (using_cloud_filters_wrong_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "'~cloud_filters/cloud_filter_chain' param in scan_to_cloud_filter_chain is incorrect.");
      RCLCPP_WARN(laser_filters_logger,
        "Please Replace with '~cloud_filter_chain'");
    }

    if (using_scan_filters_wrong_deprecated_) {
      RCLCPP_WARN(laser_filters_logger,
        "'~scan_filters/scan_filter_chain' param in scan_to_scan_filter_chain is incorrect.");
      RCLCPP_WARN(laser_filters_logger,
        "Please Replace with '~scan_filter_chain'");
    }
  }

  void scanCallback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & scan_msg)
  {
    //    sensor_msgs::msg::LaserScan scan_msg = *scan_in;

    sensor_msgs::msg::LaserScan filtered_scan;
    scan_filter_chain_.update(*scan_msg, filtered_scan);

    // Project laser into point cloud
    sensor_msgs::msg::PointCloud2 scan_cloud;

    // TODO(Rohit): CLEAN UP HACK
    // This is a trial at correcting for incident angles.
    // It makes many assumptions that do not generalise
    if (incident_angle_correction_) {
      for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
        double angle = filtered_scan.angle_min + i * filtered_scan.angle_increment;
        filtered_scan.ranges[i] = filtered_scan.ranges[i] + 0.03 * exp(-fabs(sin(angle)));
      }
    }

    // Transform into a PointCloud message
    int mask = laser_geometry::channel_option::Intensity |
      laser_geometry::channel_option::Distance |
      laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Timestamp;

    if (high_fidelity_) {
      try {
        projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, buffer_,
          mask);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(laser_filters_logger,
          "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
          target_frame_.c_str(), ex.what());
        return;
        // projector_.projectLaser(filtered_scan,scan_cloud,laser_max_range_,preservative_,mask);
      }
    } else {
      projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, buffer_,
        laser_max_range_, mask);
    }

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    cloud_filter_chain_.update(scan_cloud, filtered_cloud);

    cloud_pub_->publish(filtered_cloud);
  }
};


int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_to_cloud_filter_chain");
  ScanToCloudFilterChain f(nh);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }

  return 0;
}
