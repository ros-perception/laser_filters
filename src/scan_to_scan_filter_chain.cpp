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


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"

class ScanToScanFilterChain
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Components for tf::MessageFilter
  tf::TransformListener *tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
  double tf_filter_tolerance_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::LaserScan msg_;
  ros::Publisher output_pub_;

  // Deprecation helpers
  ros::Timer deprecation_timer_;
  bool  using_filter_chain_deprecated_;

public:
  // Constructor
  ScanToScanFilterChain() :
    private_nh_("~"),
    scan_sub_(nh_, "scan", 50),
    tf_(NULL),
    tf_filter_(NULL),
    filter_chain_("sensor_msgs::LaserScan")
  {
    // Configure filter chain
    
    using_filter_chain_deprecated_ = private_nh_.hasParam("filter_chain");

    if (using_filter_chain_deprecated_)
      filter_chain_.configure("filter_chain", private_nh_);
    else
      filter_chain_.configure("scan_filter_chain", private_nh_);
    
    std::string tf_message_filter_target_frame;

    if (private_nh_.hasParam("tf_message_filter_target_frame"))
    {
      private_nh_.getParam("tf_message_filter_target_frame", tf_message_filter_target_frame);

      private_nh_.param("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_ = new tf::TransformListener();
      tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, *tf_, "", 50);
      tf_filter_->setTargetFrame(tf_message_filter_target_frame);
      tf_filter_->setTolerance(ros::Duration(tf_filter_tolerance_));

      // Setup tf::MessageFilter generates callback
      tf_filter_->registerCallback(boost::bind(&ScanToScanFilterChain::callback, this, _1));
    }
    else 
    {
      // Pass through if no tf_message_filter_target_frame
      scan_sub_.registerCallback(boost::bind(&ScanToScanFilterChain::callback, this, _1));
    }
    
    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1000);

    // Set up deprecation printout
    deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&ScanToScanFilterChain::deprecation_warn, this, _1));
  }

  // Destructor
  ~ScanToScanFilterChain()
  {
    if (tf_filter_)
      delete tf_filter_;
    if (tf_)
      delete tf_;
  }
  
  // Deprecation warning callback
  void deprecation_warn(const ros::TimerEvent& e)
  {
    if (using_filter_chain_deprecated_)
      ROS_WARN("Use of '~filter_chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~scan_filter_chain'.");
  }

  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
    // Run the filter chain
    if (filter_chain_.update(*msg_in, msg_))
    {
      //only publish result if filter succeeded
      output_pub_.publish(msg_);
    } else {
      ROS_ERROR_THROTTLE(1, "Filtering the scan from time %i.%i failed.", msg_in->header.stamp.sec, msg_in->header.stamp.nsec);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_scan_filter_chain");
  
  ScanToScanFilterChain t;
  ros::spin();
  
  return 0;
}
