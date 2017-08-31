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


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class LaserScanRangeFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  double lower_threshold_ ;
  double upper_threshold_ ;
  bool use_message_range_limits_ ;
  float lower_replacement_value_ ;
  float upper_replacement_value_ ;

  bool configure()
  {
    use_message_range_limits_ = false;
    getParam("use_message_range_limits", use_message_range_limits_);

    // work around the not implemented getParam(std::string name, float& value) method
    double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    getParam("lower_replacement_value", temp_replacement_value);
    lower_replacement_value_ = static_cast<float>(temp_replacement_value);

    // work around the not implemented getParam(std::string name, float& value) method
    temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    getParam("upper_replacement_value", temp_replacement_value);
    upper_replacement_value_ = static_cast<float>(temp_replacement_value);


    lower_threshold_ = 0.0;
    upper_threshold_ = 100000.0;
    getParam("lower_threshold", lower_threshold_);
    getParam("upper_threshold", upper_threshold_) ;
    return true;
  }

  virtual ~LaserScanRangeFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    if (use_message_range_limits_)
    {
      lower_threshold_ = input_scan.range_min;
      upper_threshold_ = input_scan.range_max;
    }
    filtered_scan = input_scan;
    for (unsigned int i=0;
         i < input_scan.ranges.size();
         i++) // Need to check ever reading in the current scan
    {

      if (filtered_scan.ranges[i] <= lower_threshold_)
      {
        filtered_scan.ranges[i] = lower_replacement_value_;

      }
      else if (filtered_scan.ranges[i] >= upper_threshold_)
      {
        filtered_scan.ranges[i] = upper_replacement_value_;
      }
    }

    return true;
  }
} ;

}

#endif // LASER_SCAN_RANGE_FILTER_H
