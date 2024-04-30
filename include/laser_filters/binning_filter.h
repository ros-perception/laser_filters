/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2024, Anthony Goeckner <anthony.goeckner@northwestern.edu>
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

#ifndef LASER_SCAN_BINNING_FILTER_H
#define LASER_SCAN_BINNING_FILTER_H
/**
\author Anthony Goeckner
@b LaserScanBinningFilter splits input scans into bins, ensuring that every output scan has the exact same number of readings as the number of bins.

**/

#include "filters/filter_base.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{

class LaserScanBinningFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:

  uint num_bins_;

  bool configure()
  {
    num_bins_ = 360;
    getParam("num_bins", num_bins_);

    return true;
  }

  virtual ~LaserScanBinningFilter()
  { 
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    // Copy data first.
    filtered_scan = input_scan;

    // Set up filtered_scan.
    filtered_scan.ranges.assign(num_bins_, std::numeric_limits<float>::quiet_NaN());
    filtered_scan.intensities.assign(num_bins_, std::numeric_limits<float>::quiet_NaN());
    filtered_scan.angle_increment = 2.0 * M_PI / static_cast<float>(num_bins_);

    unsigned int i = 0;
    while(i < input_scan.ranges.size())
    {
      // Calculate angle.
      double angle = fmodf(input_scan.angle_min + input_scan.angle_increment * i, 2.0 * M_PI);

      // Calculate the bin index.
      unsigned int bin_index = static_cast<unsigned int>(floor(angle / filtered_scan.angle_increment));

      // Copy the range and intensity values.
      filtered_scan.ranges[bin_index] = input_scan.ranges[i];
      filtered_scan.intensities[bin_index] = input_scan.intensities[i];

      i ++;
    }
    return true;
  }
};

}

#endif // LASER_SCAN_BINNING_FILTER_H
