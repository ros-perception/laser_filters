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

/** @b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class ScanToCloudFilterChain
{
public:

  // ROS related
  laser_geometry::LaserProjection projector_; // Used to project laser scans

  double laser_max_range_;           // Used in laser scan projection
  int window_;
    
  bool high_fidelity_;                    // High fidelity (interpolating time across scan)
  std::string target_frame_;                   // Target frame for high fidelity result
  std::string scan_topic_, cloud_topic_;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
    
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

  ////////////////////////////////////////////////////////////////////////////////
  ScanToCloudFilterChain () : laser_max_range_ (DBL_MAX), private_nh("~"), filter_(tf_, "", 50),
                   cloud_filter_chain_("sensor_msgs::PointCloud2"), scan_filter_chain_("sensor_msgs::LaserScan")
  {
    private_nh.param("high_fidelity", high_fidelity_, false);
    private_nh.param("notifier_tolerance", tf_tolerance_, 0.03);
    private_nh.param("target_frame", target_frame_, std::string ("base_link"));

    // DEPRECATED with default value
    using_default_target_frame_deprecated_ = !private_nh.hasParam("target_frame");

    // DEPRECATED
    using_scan_topic_deprecated_ = private_nh.hasParam("scan_topic");
    using_cloud_topic_deprecated_ = private_nh.hasParam("cloud_topic");
    using_laser_max_range_deprecated_ = private_nh.hasParam("laser_max_range");
    using_filter_window_deprecated_ = private_nh.hasParam("filter_window");
    using_cloud_filters_deprecated_ = private_nh.hasParam("cloud_filters/filter_chain");
    using_scan_filters_deprecated_ = private_nh.hasParam("scan_filters/filter_chain");
    using_cloud_filters_wrong_deprecated_ = private_nh.hasParam("cloud_filters/cloud_filter_chain");
    using_scan_filters_wrong_deprecated_ = private_nh.hasParam("scan_filters/scan_filter_chain");


    private_nh.param("filter_window", window_, 2);
    private_nh.param("laser_max_range", laser_max_range_, DBL_MAX);
    private_nh.param("scan_topic", scan_topic_, std::string("tilt_scan"));
    private_nh.param("cloud_topic", cloud_topic_, std::string("tilt_laser_cloud_filtered"));
    private_nh.param("incident_angle_correction", incident_angle_correction_, true);

    filter_.setTargetFrame(target_frame_);
    filter_.registerCallback(boost::bind(&ScanToCloudFilterChain::scanCallback, this, _1));
    filter_.setTolerance(ros::Duration(tf_tolerance_));

    if (using_scan_topic_deprecated_)
      sub_.subscribe(nh, scan_topic_, 50);
    else
      sub_.subscribe(nh, "scan", 50);

    filter_.connectInput(sub_);

    if (using_cloud_topic_deprecated_)
      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> (cloud_topic_, 10);
    else
      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 10);

    std::string cloud_filter_xml;

    if (using_cloud_filters_deprecated_)
      cloud_filter_chain_.configure("cloud_filters/filter_chain", private_nh);
    else if (using_cloud_filters_wrong_deprecated_)
      cloud_filter_chain_.configure("cloud_filters/cloud_filter_chain", private_nh);
    else
      cloud_filter_chain_.configure("cloud_filter_chain", private_nh);

    if (using_scan_filters_deprecated_)
      scan_filter_chain_.configure("scan_filter/filter_chain", private_nh);
    else if (using_scan_filters_wrong_deprecated_)
      scan_filter_chain_.configure("scan_filters/scan_filter_chain", private_nh);
    else
      scan_filter_chain_.configure("scan_filter_chain", private_nh);

    deprecation_timer_ = nh.createTimer(ros::Duration(5.0), boost::bind(&ScanToCloudFilterChain::deprecation_warn, this, _1));
  }

  // We use a deprecation warning on a timer to avoid warnings getting lost in the noise
  void deprecation_warn(const ros::TimerEvent& e)
  {
    if (using_scan_topic_deprecated_)
      ROS_WARN("Use of '~scan_topic' parameter in scan_to_cloud_filter_chain has been deprecated.");

    if (using_cloud_topic_deprecated_)
      ROS_WARN("Use of '~cloud_topic' parameter in scan_to_cloud_filter_chain has been deprecated.");

    if (using_laser_max_range_deprecated_)
      ROS_WARN("Use of '~laser_max_range' parameter in scan_to_cloud_filter_chain has been deprecated.");

    if (using_filter_window_deprecated_)
      ROS_WARN("Use of '~filter_window' parameter in scan_to_cloud_filter_chain has been deprecated.");

    if (using_default_target_frame_deprecated_)
      ROS_WARN("Use of default '~target_frame' parameter in scan_to_cloud_filter_chain has been deprecated.  Default currently set to 'base_link' please set explicitly as appropriate.");

    if (using_cloud_filters_deprecated_)
      ROS_WARN("Use of '~cloud_filters/filter_chain' parameter in scan_to_cloud_filter_chain has been deprecated.  Replace with '~cloud_filter_chain'");

    if (using_scan_filters_deprecated_)
      ROS_WARN("Use of '~scan_filters/filter_chain' parameter in scan_to_cloud_filter_chain has been deprecated.  Replace with '~scan_filter_chain'");

    if (using_cloud_filters_wrong_deprecated_)
      ROS_WARN("Use of '~cloud_filters/cloud_filter_chain' parameter in scan_to_cloud_filter_chain is incorrect.  Please Replace with '~cloud_filter_chain'");

    if (using_scan_filters_wrong_deprecated_)
      ROS_WARN("Use of '~scan_filters/scan_filter_chain' parameter in scan_to_scan_filter_chain is incorrect.  Please Replace with '~scan_filter_chain'");

  }


  ////////////////////////////////////////////////////////////////////////////////
  void
  scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    //    sensor_msgs::LaserScan scan_msg = *scan_in;

    sensor_msgs::LaserScan filtered_scan;
    scan_filter_chain_.update (*scan_msg, filtered_scan);

    // Project laser into point cloud
    sensor_msgs::PointCloud2 scan_cloud;

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
        projector_.transformLaserScanToPointCloud (target_frame_, filtered_scan, scan_cloud, tf_, mask);
      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", target_frame_.c_str (), ex.what ());
        return;
        //projector_.projectLaser (filtered_scan, scan_cloud, laser_max_range_, preservative_, mask);
      }
    }
    else
    {
      projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, tf_, laser_max_range_, mask);
    }
      
    sensor_msgs::PointCloud2 filtered_cloud;
    cloud_filter_chain_.update (scan_cloud, filtered_cloud);

    cloud_pub_.publish(filtered_cloud);
  }

} ;



int
main (int argc, char** argv)
{
  ros::init (argc, argv, "scan_to_cloud_filter_chain");
  ros::NodeHandle nh;
  ScanToCloudFilterChain f;

  ros::spin();

  return (0);
}


