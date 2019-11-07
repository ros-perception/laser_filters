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
#ifndef LASER_FILTERS_SCAN_TO_CLOUD_FILTER_CHAIN_H
#define LASER_FILTERS_SCAN_TO_CLOUD_FILTER_CHAIN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <float.h>

// Laser projection
#include <laser_geometry/laser_geometry.h>

// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_chain.h"

namespace laser_filters
{

/** @b ScanShadowsFilter is a simple class that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class ScanToCloudFilterChain
{
public:
  ScanToCloudFilterChain (const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

protected:

  // ROS related
  laser_geometry::LaserProjection projector_; // Used to project laser scans

  double laser_max_range_;           // Used in laser scan projection
  int window_;
    
  bool high_fidelity_;                    // High fidelity (interpolating time across scan)
  std::string target_frame_;                   // Target frame for high fidelity result
  std::string scan_topic_, cloud_topic_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
    
  // TF
  tf::TransformListener tf_;

  message_filters::Subscriber<sensor_msgs::LaserScan> sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> filter_;

  double tf_tolerance_;
  filters::FilterChain<sensor_msgs::PointCloud2> cloud_filter_chain_;
  filters::FilterChain<sensor_msgs::LaserScan> scan_filter_chain_;
  ros::Publisher cloud_pub_;

  // Timer for displaying deprecation warnings
  ros::Timer deprecation_timer_;
  bool  using_scan_topic_deprecated_;
  bool  using_cloud_topic_deprecated_;
  bool  using_default_target_frame_deprecated_;
  bool  using_laser_max_range_deprecated_;
  bool  using_filter_window_deprecated_;
  bool  using_scan_filters_deprecated_;
  bool  using_cloud_filters_deprecated_;
  bool  using_scan_filters_wrong_deprecated_;
  bool  using_cloud_filters_wrong_deprecated_;
  bool  incident_angle_correction_;

  // We use a deprecation warning on a timer to avoid warnings getting lost in the noise
  void deprecation_warn(const ros::TimerEvent& e);
  
  ////////////////////////////////////////////////////////////////////////////////
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};

}  // namespace laser_filters

#endif  // LASER_FILTERS_SCAN_TO_CLOUD_FILTER_CHAIN_H
