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

#include <laser_filters/intensity_filter.h>

namespace laser_filters
{
LaserScanIntensityFilter::LaserScanIntensityFilter()
{
}

bool LaserScanIntensityFilter::configure()
{
  // dynamic reconfigure parameters callback:
  on_set_parameters_callback_handle_ = params_interface_->add_on_set_parameters_callback(
            std::bind(&LaserScanIntensityFilter::reconfigureCB, this, std::placeholders::_1));

  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("lower_threshold"), lower_threshold_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanIntensityFilter was not given lower_threshold.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("upper_threshold"), upper_threshold_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanIntensityFilter was not given upper_threshold.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("invert"), invert_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanIntensityFilter was not given invert.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_override_range"), filter_override_range_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanIntensityFilter was not given filter_override_range.\n");
    return false;
  }  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_override_intensity"), filter_override_intensity_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanIntensityFilter was not given filter_override_intensity.\n");
    return false;
  }
  return true;
}

bool LaserScanIntensityFilter::update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
{
  auto start = std::chrono::high_resolution_clock::now();

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

  auto end = std::chrono::high_resolution_clock::now();
  auto update_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  RCLCPP_DEBUG(logging_interface_->get_logger(), "LaserScanIntensityFilter update took %lu microseconds", update_elapsed);
  
  return true;
}

rcl_interfaces::msg::SetParametersResult LaserScanIntensityFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == "lower_threshold"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          lower_threshold_ = parameter.as_double();
      else if(parameter.get_name() == "upper_threshold" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          upper_threshold_ = parameter.as_double();
      else if(parameter.get_name() == "invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          invert_ = parameter.as_bool();
      else if(parameter.get_name() == "filter_override_range" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          filter_override_range_ = parameter.as_bool();
      else if(parameter.get_name() == "filter_override_intensity" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          filter_override_intensity_ = parameter.as_bool();
      else
        RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
    }
  return result;
}
}