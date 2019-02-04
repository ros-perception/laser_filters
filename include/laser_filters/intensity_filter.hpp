/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef LASER_FILTERS__INTENSITY_FILTER_HPP_
#define LASER_FILTERS__INTENSITY_FILTER_HPP_
/**
\author Vijay Pradeep
@b ScanIntensityFilter takes input scans and fiters out that are not within the specified range. The filtered out readings are set at >max_range in order to invalidate them.

**/

#include <cmath>  // for isnan()
#include <limits>
#include "filters/filter_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define NUM_BUCKETS 24
namespace laser_filters
{

class LaserScanIntensityFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  double lower_threshold_;
  double upper_threshold_;
  int disp_hist_;
  bool disp_hist_enabled_;

  bool configure()
  {
    // Get the parameter value, If the parameter was not set, then assign default value.
    node_->get_parameter_or("params.lower_threshold", lower_threshold_, 8000.0);
    node_->get_parameter_or("params.upper_threshold", upper_threshold_, 100000.0);
    node_->get_parameter_or("params.disp_histogram", disp_hist_, 1);

    disp_hist_enabled_ = (disp_hist_ == 0) ? false : true;

    return true;
  }

  LaserScanIntensityFilter() {}

  virtual ~LaserScanIntensityFilter() {}

  bool update(
    const sensor_msgs::msg::LaserScan & input_scan,
    sensor_msgs::msg::LaserScan & filtered_scan)
  {
    const double hist_max = 4 * 12000.0;
    // const int num_buckets = 24;
    // int histogram[num_buckets];
    int histogram[NUM_BUCKETS];
    for (int i = 0; i < NUM_BUCKETS; i++) {
      histogram[i] = 0;
    }

    filtered_scan = input_scan;

    // Need to check ever reading in the current scan
    for (unsigned int i = 0;
      i < input_scan.ranges.size() && i < input_scan.intensities.size();
      i++)
    {
      // Is this reading below our lower threshold?
      // Is this reading above our upper threshold?
      if (filtered_scan.intensities[i] <= lower_threshold_ ||
        filtered_scan.intensities[i] >= upper_threshold_)
      {
        // If so, then make it an invalid value (NaN)
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }

      // Calculate histogram
      if (disp_hist_enabled_) {
        // If intensity value is inf or NaN, skip voting histogram
        if (std::isinf(static_cast<double>(filtered_scan.intensities[i])) ||
          std::isnan(static_cast<double>(filtered_scan.intensities[i])) )
        {
          continue;
        }

        // Choose bucket to vote on histogram,
        // and check the index of bucket is in the histogram array
        int cur_bucket = static_cast<int>(filtered_scan.intensities[i] / hist_max * NUM_BUCKETS);
        if (cur_bucket > NUM_BUCKETS - 1) {
          cur_bucket = NUM_BUCKETS - 1;
        } else if (cur_bucket < 0) {cur_bucket = 0;}
        histogram[cur_bucket]++;
      }
    }

    // Display Histogram
    if (disp_hist_enabled_) {
      printf("********** SCAN **********\n");
      for (int i = 0; i < NUM_BUCKETS; i++) {
        printf("%u - %u: %u\n", (unsigned int) hist_max / NUM_BUCKETS * i,
          (unsigned int) hist_max / NUM_BUCKETS * (i + 1),
          histogram[i]);
      }
    }
    return true;
  }
};
}  // namespace laser_filters

#endif  // LASER_FILTERS__INTENSITY_FILTER_HPP_
