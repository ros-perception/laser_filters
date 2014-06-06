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

  bool configure()
  {
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
    filtered_scan = input_scan;
    for (unsigned int i=0;
         i < input_scan.ranges.size();
         i++) // Need to check ever reading in the current scan
    {
      {
      if (filtered_scan.ranges[i] <= lower_threshold_ || filtered_scan.ranges[i] >= upper_threshold_)
        {
          filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }

    return true;
  }
} ;

}

#endif // LASER_SCAN_RANGE_FILTER_H
