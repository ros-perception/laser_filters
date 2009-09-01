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
#include <message_filters/subscriber.h>
#include "tf/message_filter.h"

#include <tf/transform_listener.h>

#include "filters/filter_chain.h"


class GenericLaserScanFilterNode 
{
public:
  GenericLaserScanFilterNode() 
    :  filter_chain_("sensor_msgs::LaserScan")
    , scan_sub_(nh_, "scan_in", 50)
						  
  {
    ros::NodeHandle local_nh("~");
    output_pub_ = local_nh.advertise<sensor_msgs::LaserScan>("output", 1000);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, tf_, "base_link", 50);
    tf_filter_->registerCallback(boost::bind(&GenericLaserScanFilterNode::callback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.03));

    filter_chain_.configure("~");
  }

  ~GenericLaserScanFilterNode(){
    delete tf_filter_;
  }

  void callback(const sensor_msgs::LaserScanConstPtr& msg_in)
  {
    filter_chain_.update (*msg_in, msg_);
    output_pub_.publish(msg_);
  }

protected:
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* tf_filter_;
  ros::Publisher output_pub_;
  sensor_msgs::LaserScan msg_;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_filter_node");
  
  GenericLaserScanFilterNode t;
  ros::spin();
  
  return 0;
}

