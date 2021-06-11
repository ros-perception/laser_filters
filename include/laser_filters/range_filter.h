/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, JSK (The University of Tokyo).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_RANGE_FILTER_H
#define LASER_SCAN_RANGE_FILTER_H
/**
\author Yohei Kakiuchi
@b ScanRangeFilter takes input scans and filters within indicated range.
**/


#include <dynamic_reconfigure/server.h>
#include <laser_filters/RangeFilterConfig.h>
#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class LaserScanRangeFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  std::shared_ptr<dynamic_reconfigure::Server<RangeFilterConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;

  RangeFilterConfig config_ = RangeFilterConfig::__getDefault__();

public:
  bool configure()
  {
    ros::NodeHandle private_nh("~" + getName());
    dyn_server_.reset(new dynamic_reconfigure::Server<RangeFilterConfig>(own_mutex_, private_nh));
    dynamic_reconfigure::Server<RangeFilterConfig>::CallbackType f;
    f = boost::bind(&LaserScanRangeFilter::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(f);

    getParam("lower_threshold", config_.lower_threshold);
    getParam("upper_threshold", config_.upper_threshold);
    getParam("use_message_range_limits", config_.use_message_range_limits);
    getParam("lower_replacement_value", config_.lower_replacement_value);
    getParam("upper_replacement_value", config_.upper_replacement_value);

    dyn_server_->updateConfig(config_);
    return true;
  }

  virtual ~LaserScanRangeFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    double lower_threshold = config_.lower_threshold;
    double upper_threshold = config_.upper_threshold;

    if (config_.use_message_range_limits)
    {
      lower_threshold = input_scan.range_min;
      upper_threshold = input_scan.range_max;
    }
    filtered_scan = input_scan;
    for (unsigned int i=0;
         i < input_scan.ranges.size();
         i++) // Need to check ever reading in the current scan
    {

      if (filtered_scan.ranges[i] <= lower_threshold)
      {
        filtered_scan.ranges[i] = config_.lower_replacement_value;

      }
      else if (filtered_scan.ranges[i] >= upper_threshold)
      {
        filtered_scan.ranges[i] = config_.upper_replacement_value;
      }
    }

    return true;
  }

  void reconfigureCB(RangeFilterConfig& config, uint32_t level)
  {
    config_ = config;
  }
} ;

}

#endif // LASER_SCAN_RANGE_FILTER_H
