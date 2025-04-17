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

#ifndef LASER_SCAN_INTENSITY_FILTER_H
#define LASER_SCAN_INTENSITY_FILTER_H
/**
\author Vijay Pradeep
@b ScanIntensityFilter takes input scans and fiters out that are not within the specified range. The filtered out readings are set at >max_range in order to invalidate them.

**/

#include "filters/filter_base.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{

class LaserScanIntensityFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:

  double lower_threshold_ ;
  double upper_threshold_ ;
  int disp_hist_ ;
  bool disp_hist_enabled_;
  bool invert_;
  bool filter_override_range_;
  bool filter_override_intensity_;

  bool configure()
  {
    getParam("lower_threshold", lower_threshold_, false, 8000.0);
    getParam("upper_threshold", upper_threshold_, false, 100000.0) ;
    getParam("disp_histogram",  disp_hist_, false, 1) ;
    getParam("invert", invert_, false, false);
    getParam("filter_override_range", filter_override_range_, false, true);
    getParam("filter_override_intensity", filter_override_intensity_, false, false);

    disp_hist_enabled_ = (disp_hist_ == 0) ? false : true;

    return true;
  }

  virtual ~LaserScanIntensityFilter(){}

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    const double hist_max = 4*12000.0 ;
    const int num_buckets = 24 ;
    int histogram[num_buckets] ;
    for (int i=0; i < num_buckets; i++)
      histogram[i] = 0 ;

    filtered_scan = input_scan;

    // Need to check ever reading in the current scan
    for (unsigned int i=0;
         i < input_scan.ranges.size() && i < input_scan.intensities.size();
         i++)
    {
      // Is this reading below our lower threshold?
      // Is this reading above our upper threshold?
      bool filter = filtered_scan.intensities[i] <= lower_threshold_ || filtered_scan.intensities[i] >= upper_threshold_;

      if (invert_)
      {
        filter = !filter;
      }

      if (filter)
      {
        if (filter_override_range_)
        {
          // If so, then make it an invalid value (NaN)
          filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
        if (filter_override_intensity_)
        {
          filtered_scan.intensities[i] = 0.0;  // Not intense
        }
      }
      else
      {
          if (filter_override_intensity_)
          {
            filtered_scan.intensities[i] = 1.0;  // Intense
          }
      }

      // Calculate histogram
      if (disp_hist_enabled_){
        // If intensity value is inf or NaN, skip voting histogram
        if( std::isinf((double)filtered_scan.intensities[i]) ||
            std::isnan((double)filtered_scan.intensities[i]) )
          continue;

        // Choose bucket to vote on histogram,
        // and check the index of bucket is in the histogram array
        int cur_bucket = (int)(filtered_scan.intensities[i] / hist_max * num_buckets);
        if (cur_bucket > num_buckets-1)
          cur_bucket = num_buckets-1;
        else if (cur_bucket < 0) cur_bucket = 0;
        histogram[cur_bucket]++;
      }
    }

    // Display Histogram
    if (disp_hist_enabled_)
    {
      printf("********** SCAN **********\n") ;
      for (int i=0; i < num_buckets; i++)
      {
        printf("%u - %u: %u\n", (unsigned int) hist_max/num_buckets*i,
                                (unsigned int) hist_max/num_buckets*(i+1),
                                histogram[i]) ;
      }
    }
    return true;
  }

  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters)
  {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      for (auto parameter : parameters)
      {
          if(logging_interface_ != nullptr)
              RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
          if(parameter.get_name() == param_prefix_ + "lower_threshold"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              lower_threshold_ = parameter.as_double();
          else if(parameter.get_name() == param_prefix_ + "upper_threshold" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              upper_threshold_ = parameter.as_double();
          else if(parameter.get_name() == param_prefix_ + "disp_hist" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
              disp_hist_ = parameter.as_int();
          else if(parameter.get_name() == param_prefix_ + "invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
              invert_ = parameter.as_bool();
          else if(parameter.get_name() == param_prefix_ + "filter_override_range" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
              filter_override_range_ = parameter.as_bool();
          else if(parameter.get_name() == param_prefix_ + "filter_override_intensity" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
              filter_override_intensity_ = parameter.as_bool();
          else
          if(logging_interface_ != nullptr) RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
      }

      return result;
  }
};

}

#endif // LASER_SCAN_INTENSITY_FILTER_H
