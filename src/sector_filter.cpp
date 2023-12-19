/*********************************************************************
* BSD 2-Clause License
*
* Copyright (c) 2021, Jimmy F. Klarke
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author: Jimmy F. Klarke
*********************************************************************/

#include <math.h>

#include <laser_filters/sector_filter.h>

namespace laser_filters
{

LaserScanSectorFilter::LaserScanSectorFilter()
{
}

bool LaserScanSectorFilter::configure()
{
  // dynamic reconfigure parameters callback:
  on_set_parameters_callback_handle_ = params_interface_->add_on_set_parameters_callback(
            std::bind(&LaserScanSectorFilter::reconfigureCB, this, std::placeholders::_1));

  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("angle_min"), angle_min_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given angle_min.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("angle_max"), angle_max_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given angle_max.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("range_min"), range_min_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given range_min.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("range_max"), range_max_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given range_max.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("clear_inside"), clear_inside_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given clear_inside.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("invert"), invert_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given invert.\n");
    return false;
  }

  RCLCPP_INFO(logging_interface_->get_logger(), "clear_inside(!invert): %s", (isClearInside() ? "true" : "false"));
  return true;
}

rcl_interfaces::msg::SetParametersResult LaserScanSectorFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == "angle_min"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          angle_min_ = parameter.as_double();
      else if(parameter.get_name() == "angle_max" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          angle_max_ = parameter.as_double();
      else if(parameter.get_name() == "range_min" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          range_min_ = parameter.as_double();
      else if(parameter.get_name() == "range_max" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          range_max_ = parameter.as_double();
      else if(parameter.get_name() == "clear_inside" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          clear_inside_ = parameter.as_bool();
      else if(parameter.get_name() == "invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          invert_ = parameter.as_bool();
      else
        RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
    }
  return result;
}

bool LaserScanSectorFilter::isClearInside()
{
  return invert_ ? false : clear_inside_;
}

bool LaserScanSectorFilter::update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
{
  filtered_scan = input_scan; //copy entire message
  bool clear_inside = isClearInside();

  double angle_delta = angle_max_ - angle_min_;
  if (angle_max_ < angle_min_)
  {
    angle_delta += M_PI * 2;
  }

  double current_angle = input_scan.angle_min;
  unsigned int count = 0;
  //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
  for (size_t i = 0; i < input_scan.ranges.size(); ++i)
  {
    current_angle = (i == 0) ? current_angle : (current_angle + input_scan.angle_increment);

    double current_range = input_scan.ranges[i];
    double current_angle_delta = current_angle - angle_min_;
    if ((angle_max_ < angle_min_) && (current_angle_delta < 0))
    {
      current_angle_delta += M_PI * 2;
    }

    if (clear_inside != ((current_angle_delta > 0)
                      && (current_angle_delta < angle_delta)
                      && (current_range > range_min_)
                      && (current_range < range_max_)))
    {
      continue;
    }

    filtered_scan.ranges[i] = input_scan.range_max + 1.0;
    if (i < filtered_scan.intensities.size())
    {
      filtered_scan.intensities[i] = 0.0;
    }
    count++;
  }

  RCLCPP_DEBUG(logging_interface_->get_logger(), "Filtered out %u points from the laser scan.", count);

  return true;
}

} // end namespace laser_filters
