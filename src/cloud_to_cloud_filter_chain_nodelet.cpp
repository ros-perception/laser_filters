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


#include <pluginlib/class_list_macros.h>
#include "laser_filters/cloud_to_cloud_filter_chain_nodelet.h"

PLUGINLIB_EXPORT_CLASS(laser_filters::CloudToCloudFilterChain, nodelet::Nodelet)

namespace laser_filters {

// Constructor
  void CloudToCloudFilterChain::onInit()

    /*scan_sub_(nh_, "scan", 50),
    tf_(NULL),
    tf_filter_(NULL),
    filter_chain_("sensor_msgs::LaserScan")*/
  {
    nh_= getNodeHandle();
    ROS_ERROR("Creating node hangle");
    private_nh_ = getPrivateNodeHandle();
    scan_sub_.subscribe(nh_, "cloud_in", 50);

    filter_chain_ = new filters::FilterChain<sensor_msgs::PointCloud2>("sensor_msgs::PointCloud2");
    private_nh_.param("filter_chain", using_filter_chain_deprecated_);

    if (using_filter_chain_deprecated_) {
      filter_chain_->configure("filter_chain", private_nh_);
    } else {
      filter_chain_->configure("scan_filter_chain", private_nh_);
    }

    std::string tf_message_filter_target_frame;
    ROS_ERROR("Has param");
    if (private_nh_.hasParam("tf_message_filter_target_frame"))
    {
      private_nh_.getParam("tf_message_filter_target_frame", tf_message_filter_target_frame);

      private_nh_.param("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_ = new tf::TransformListener();
      tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(scan_sub_, *tf_, "", 50);
      tf_filter_->setTargetFrame(tf_message_filter_target_frame);
      tf_filter_->setTolerance(ros::Duration(tf_filter_tolerance_));

      // Setup tf::MessageFilter generates callback
      tf_filter_->registerCallback(boost::bind(&CloudToCloudFilterChain::callback, this, _1));
    }
    else 
    {
      // Pass through if no tf_message_filter_target_frame
      scan_sub_.registerCallback(boost::bind(&CloudToCloudFilterChain::callback, this, _1));
    }
    
    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1000);

    // Set up deprecation printout
    deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&CloudToCloudFilterChain::deprecation_warn, this, _1));
    ROS_ERROR("End on Init");
  }

  CloudToCloudFilterChain::CloudToCloudFilterChain()
  {

  }
  // Destructor
  CloudToCloudFilterChain::~CloudToCloudFilterChain()
  {
    if (tf_filter_)
      delete tf_filter_;
    if (tf_)
      delete tf_;
  }
  
  // Deprecation warning callback
  void CloudToCloudFilterChain::deprecation_warn(const ros::TimerEvent& e)
  {
    if (using_filter_chain_deprecated_)
      ROS_WARN("Use of '~filter_chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~scan_filter_chain'.");
  }

  // Callback
  void CloudToCloudFilterChain::callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
  {
    // Run the filter chain
    if (filter_chain_->update(*msg_in, msg_))
    {
      //only publish result if filter succeeded
      output_pub_.publish(msg_);
    }
  }
};

