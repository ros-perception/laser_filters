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

#include <float.h>

// Laser projection
#include <laser_geometry/laser_geometry.hpp>

//Filters
#include "filters/filter_chain.hpp"

/** @b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class ScanToCloudFilterChain
{
public:

  // ROS related
  laser_geometry::LaserProjection projector_; // Used to project laser scans

  rclcpp::Node::SharedPtr nh_;
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

  ////////////////////////////////////////////////////////////////////////////////
  ScanToCloudFilterChain(rclcpp::Node::SharedPtr node) : nh_(node),
                                                         laser_max_range_(DBL_MAX),
                                                         buffer_(nh_->get_clock()),
                                                         tf_(buffer_),
                                                         sub_(nh_, "scan", rmw_qos_profile_sensor_data),
                                                         filter_(sub_, buffer_, "", 50, nh_),
                                                         cloud_filter_chain_("sensor_msgs::msg::PointCloud2"),
                                                         scan_filter_chain_("sensor_msgs::msg::LaserScan")
  {
    nh_->declare_parameter("high_fidelity", false);
    nh_->declare_parameter("notifier_tolerance", 0.03);
    nh_->declare_parameter("target_frame", std::string("base_link"));
    nh_->declare_parameter("incident_angle_correction", true);
    
    nh_->get_parameter("high_fidelity", high_fidelity_);
    nh_->get_parameter("notifier_tolerance", tf_tolerance_);
    nh_->get_parameter("target_frame", target_frame_);
    nh_->get_parameter("incident_angle_correction", incident_angle_correction_);

    nh_->get_parameter_or("filter_window", window_, 2);
    nh_->get_parameter_or("laser_max_range", laser_max_range_, DBL_MAX);
    nh_->get_parameter_or("scan_topic", scan_topic_, std::string("tilt_scan"));
    nh_->get_parameter_or("cloud_topic", cloud_topic_, std::string("tilt_laser_cloud_filtered"));


    filter_.setTargetFrame(target_frame_);
    filter_.registerCallback(std::bind(&ScanToCloudFilterChain::scanCallback, this, std::placeholders::_1));
    filter_.setTolerance(std::chrono::duration<double>(tf_tolerance_));
                                                           
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      nh_->get_node_base_interface(),
      nh_->get_node_timers_interface());
    buffer_.setCreateTimerInterface(timer_interface);
                                                           
    sub_.subscribe(nh_, "scan", rmw_qos_profile_sensor_data);

    filter_.connectInput(sub_);

    cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 10);

    cloud_filter_chain_.configure("cloud_filter_chain", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());

    scan_filter_chain_.configure("scan_filter_chain", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  scanCallback (const std::shared_ptr<const sensor_msgs::msg::LaserScan>& scan_msg)
  {
    //    sensor_msgs::msg::LaserScan scan_msg = *scan_in;

    sensor_msgs::msg::LaserScan filtered_scan;
    scan_filter_chain_.update (*scan_msg, filtered_scan);

    // Project laser into point cloud
    sensor_msgs::msg::PointCloud2 scan_cloud;

    //\TODO CLEAN UP HACK 
    // This is a trial at correcting for incident angles.  It makes many assumptions that do not generalise
    if(incident_angle_correction_)
    {
      for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++)
      {
        double angle = filtered_scan.angle_min + i * filtered_scan.angle_increment;
        filtered_scan.ranges[i] = filtered_scan.ranges[i] + 0.03 * exp(-fabs(sin(angle)));
      }
    }

    // Transform into a PointCloud message
    int mask = laser_geometry::channel_option::Intensity |
      laser_geometry::channel_option::Distance |
      laser_geometry::channel_option::Index |
      laser_geometry::channel_option::Timestamp;
      
    if (high_fidelity_)
    {
      try
      {
        projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, buffer_, mask);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(nh_->get_logger(), "High fidelity enabled, but TF returned a transform exception to frame %s: %s", target_frame_.c_str(), ex.what());
        return;
        //projector_.projectLaser (filtered_scan, scan_cloud, laser_max_range_, preservative_, mask);
      }
    }
    else
    {
      projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, buffer_, laser_max_range_, mask);
    }
      
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    cloud_filter_chain_.update (scan_cloud, filtered_cloud);

    cloud_pub_->publish(filtered_cloud);
  }

} ;



int
main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_to_cloud_filter_chain");
  ScanToCloudFilterChain f(nh);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {

    rclcpp::spin_some(nh);
    loop_rate.sleep();

  }

  return (0);
}
