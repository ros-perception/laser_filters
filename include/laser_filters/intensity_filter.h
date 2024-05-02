/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2020, Eurotec B.V.
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
*
*  \author Vijay Pradeep, Rein Appeldoorn
*
*********************************************************************/

#pragma once

#include "filters/filter_base.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{

class LaserScanIntensityFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:

  double lower_threshold_ ;
  double upper_threshold_ ;
  bool invert_;
  bool filter_override_range_;
  bool filter_override_intensity_;

  bool configure()
  {
    lower_threshold_ = 8000.0;
    upper_threshold_ = 100000.0;
    invert_ = false;
    filter_override_range_ = false;
    filter_override_intensity_ = true;
    getParam("lower_threshold", lower_threshold_);
    getParam("upper_threshold", upper_threshold_) ;
    getParam("invert", invert_);
    getParam("filter_override_range", filter_override_range_);
    getParam("filter_override_intensity", filter_override_intensity_);
    return true;
  }

  virtual ~LaserScanIntensityFilter(){}

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;

    // Need to check ever reading in the current scan
    for (unsigned int i=0; i < input_scan.ranges.size() && i < input_scan.intensities.size(); i++)
    {
      float& range = filtered_scan.ranges[i];
      float& intensity = filtered_scan.intensities[i];

      // Is this reading below our lower threshold?
      // Is this reading above our upper threshold?
      bool filter = intensity <= lower_threshold_ || intensity >= upper_threshold_;
      if (invert_)
      {
        filter = !filter;
      }

      if (filter)
      {
        if (filter_override_range_)
        {
          // If so, then make it an invalid value (NaN)
          range = std::numeric_limits<float>::quiet_NaN();
        }
        if (filter_override_intensity_)
        {
          intensity = 0.0;  // Not intense
        }
      }
      else
      {
        if (filter_override_intensity_)
        {
          intensity = 1.0;  // Intense
        }
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
      if(parameter.get_name() == param_prefix_+"lower_threshold"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        lower_threshold_ = parameter.as_double();
      else if(parameter.get_name() == param_prefix_+"upper_threshold" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        upper_threshold_ = parameter.as_double();
      else if(parameter.get_name() == param_prefix_+"invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        invert_ = parameter.as_bool();
      else if(parameter.get_name() == param_prefix_+"filter_override_range" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        filter_override_range_ = parameter.as_bool();
      else if(parameter.get_name() == param_prefix_+"filter_override_intensity" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        filter_override_intensity_ = parameter.as_bool();
      else{
        RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown parameter: "<<parameter.get_name());
      }
    }
    return result;
  }
};
}