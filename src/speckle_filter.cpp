/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by Eurotec B.V.
 *  Copyright (c) 2020, Eurotec B.V.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  speckle_filter.cpp
 */

#include <laser_filters/speckle_filter.h>

namespace laser_filters
{
LaserScanSpeckleFilter::LaserScanSpeckleFilter()
{
  validator_ = 0;
}

LaserScanSpeckleFilter::~LaserScanSpeckleFilter()
{
  if (!validator_)
  {
    delete validator_;
  }
}

bool LaserScanSpeckleFilter::configure()
{
  node_ = std::make_shared<rclcpp::Node>(getName());
  // dynamic reconfigure parameters callback:
  on_set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&LaserScanSpeckleFilter::reconfigureCB, this, std::placeholders::_1));

  // get params
  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_type"), filter_type_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given filter_type.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_range"), max_range_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given max_range.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_range_difference"), max_range_difference_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given max_range_difference.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_window"), filter_window_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given filter_window.\n");
    return false;
  }
  
  switch (filter_type_) {
    case laser_filters::SpeckleFilterType::RadiusOutlier:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::RadiusOutlierWindowValidator();
      break;

    case laser_filters::SpeckleFilterType::Distance:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::DistanceWindowValidator();
      break;

    default:
      break;
  }

  return true;
}

bool LaserScanSpeckleFilter::update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan)
{
  auto start = std::chrono::high_resolution_clock::now();

  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  output_scan = input_scan;

  std::vector<bool> &valid_ranges = valid_ranges_work_;

  /*Check if range size is big enough to use the filter window */
  if (output_scan.ranges.size() <= filter_window_ + 1)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Scan ranges size is too small: size = %ld", output_scan.ranges.size());
    return false;
  }

  size_t i = 0;
  size_t i_max = input_scan.ranges.size();
  valid_ranges.clear();
  while (i < i_max) {
    bool out_of_range = output_scan.ranges[i] > max_range_;
    valid_ranges.push_back(out_of_range);
    ++i;
  }

  i = 0;
  i_max = input_scan.ranges.size() - filter_window_ + 1;
  while (i < i_max) {
    bool window_valid = validator_->checkWindowValid(
      output_scan, i, filter_window_, max_range_difference_
    );
    if (window_valid) {
      size_t j = i, j_max = i + filter_window_;
      do {
        valid_ranges[j++] = true;
      } while (j < j_max);
    }
    ++i;
  }

  i = 0;
  i_max = valid_ranges.size();
  while (i < i_max) {
    if (!valid_ranges[i]) {
      output_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
    ++i;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto update_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  RCLCPP_DEBUG(logging_interface_->get_logger(), "LaserScanSpeckleFilter", "LaserScanSpeckleFilter update took %lu microseconds", update_elapsed);

  return true;
}

rcl_interfaces::msg::SetParametersResult LaserScanSpeckleFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == "filter_type"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          filter_type_ = parameter.as_int();
      else if(parameter.get_name() == "max_range" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_range_ = parameter.as_double();
      else if(parameter.get_name() == "max_range_difference" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_range_difference_ = parameter.as_double();
      else if(parameter.get_name() == "filter_window" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          filter_window_ = parameter.as_int();
      else
        RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
    }

  switch (filter_type_) {
    case laser_filters::SpeckleFilterType::RadiusOutlier:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::RadiusOutlierWindowValidator();
      break;

    case laser_filters::SpeckleFilterType::Distance:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::DistanceWindowValidator();
      break;

    default:
      break;
  }

  return result;

}

}