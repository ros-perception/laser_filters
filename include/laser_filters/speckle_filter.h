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
 *  speckle_filter.h
 */

#ifndef SPECKLE_FILTER_H
#define SPECKLE_FILTER_H

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{

enum SpeckleFilterType //Enum to select the filtering method
{
  Distance = 0, // Range based filtering (distance between consecutive points
  RadiusOutlier = 1 // Euclidean filtering based on radius outlier search
};

class WindowValidator
{
public:
  virtual bool checkWindowValid(const sensor_msgs::msg::LaserScan& scan, size_t idx, size_t window, double max_range_difference) = 0;
};

class DistanceWindowValidator : public WindowValidator
{
  virtual bool checkWindowValid(const sensor_msgs::msg::LaserScan& scan, size_t idx, size_t window, double max_range_difference)
  {
    const float& range = scan.ranges[idx];
    if (range != range) {
      return false;
    }

    for (size_t neighbor_idx_nr = 1; neighbor_idx_nr < window; ++neighbor_idx_nr)
    {
      size_t neighbor_idx = idx + neighbor_idx_nr;
      if (neighbor_idx < scan.ranges.size())  // Out of bound check
      {
        const float& neighbor_range = scan.ranges[neighbor_idx];
        if (neighbor_range != neighbor_range || fabs(neighbor_range - range) > max_range_difference)
        {
          return false;
        }
      }
    }
    return true;
  }
};

class RadiusOutlierWindowValidator : public WindowValidator
{
  virtual bool checkWindowValid(const sensor_msgs::msg::LaserScan& scan, size_t idx, size_t window, double max_distance)
  {
    int num_neighbors = 0;
    const float& r1 = scan.ranges[idx];
    float r2 = 0.;

    // Look around the current point until either the window is exceeded
    // or the number of neighbors was found.
    for (int y = -(int)window; y < (int)window + 1 && num_neighbors < (int)window; y++)
    {
      int j = idx + y;
      r2 = scan.ranges[j];

      if (j < 0 || j >= static_cast<int>(scan.ranges.size()) || idx == j || std::isnan(r2))
      {  // Out of scan bounds or itself or infinity
        continue;
      }

      // Explanation:
      //
      // Distance between two points:
      // d² = (x2 - x1)² + (y2 - y1)²
      //
      // Substitute x with r * cos(phi) and y with r * sin(phi):
      // d² = (r2 * cos(phi2) - r1 * cos(phi1))² + (r2 * sin(phi2) - r1 * sin(phi1))²
      //
      // Apply binomial theorem:
      // d² = ((r2² * cos(phi2)² + r1² * cos(phi1)² - 2 * r1 * r2 * cos(phi1) * cos(phi2)) +
      //      ((r2² * sin(phi2)² + r1² * sin(phi1)² - 2 * r1 * r2 * sin(phi1) * sin(phi2))
      //
      // Merge sums:
      // d² = r2² * (cos(phi2)² + sin(phi2)²) + r1² * (cos(phi1)² + sin(phi1)² -
      //      2 * r1 * r2 * (cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2))
      //
      // Apply cos² + sin² = 1:
      // d² = r2² + r1² - 2 * r1 * r2 * (cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2))
      //
      // Note the following:
      // cos(phi1) * cos(phi2) = 1/2 * (cos(phi1 - phi2) + cos(phi1 + phi2))
      // sin(phi1) * sin(phi2) = 1/2 * (cos(phi1 - phi2) - cos(phi1 + phi2))
      //
      // cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2) = cos(phi1 - phi2)
      //
      // Finally, phi1 - phi2 is our included_angle.

      const float d = sqrt(
            pow(r1,2) + pow(r2,2) -
            (2 * r1 * r2 * cosf(y * scan.angle_increment)));


      if (d <= max_distance)
      {
        num_neighbors++;
      }
    }

    // consider the window to be the number of neighbors we need
    if (num_neighbors < window)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
};

/**
 * @brief This is a filter that removes speckle points in a laser scan based on consecutive ranges
 */
class LaserScanSpeckleFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanSpeckleFilter()
  {
    validator_ = 0;
  }

  ~LaserScanSpeckleFilter()
  {
    if (!validator_)
    {
      delete validator_;
    }
  }

  ///////////////////////////////////////////////////////////////
  bool configure(){
    // get params
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_type"), filter_type, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given filter_type.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_range"), max_range, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given max_range.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_range_difference"), max_range_difference, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given max_range_difference.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("filter_window"), filter_window, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: SpeckleFilter was not given filter_window.\n");
      return false;
    }

    switch (filter_type) {
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
  /////////////////////////////////////////////////////
  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan){
    output_scan = input_scan;
    std::vector<bool> valid_ranges(output_scan.ranges.size(), false);

    /*Check if range size is big enough to use the filter window */
    if (output_scan.ranges.size() <= filter_window + 1)
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Scan ranges size is too small for set window: size = %i, window = %i", output_scan.ranges.size(), filter_window);
      return false;
    }

    for (size_t idx = 0; idx < output_scan.ranges.size() - filter_window + 1; ++idx)
    {
      bool window_valid = validator_->checkWindowValid(
            output_scan, idx, filter_window, max_range_difference);

      // Actually set the valid ranges (do not set to false if it was already valid or out of range)
      for (size_t neighbor_idx_or_self_nr = 0; neighbor_idx_or_self_nr < filter_window; ++neighbor_idx_or_self_nr)
      {
        size_t neighbor_idx_or_self = idx + neighbor_idx_or_self_nr;
        if (neighbor_idx_or_self < output_scan.ranges.size())  // Out of bound check
        {
          bool out_of_range = output_scan.ranges[neighbor_idx_or_self] > max_range;
          valid_ranges[neighbor_idx_or_self] = valid_ranges[neighbor_idx_or_self] || window_valid || out_of_range;
        }
      }
    }

    for (size_t idx = 0; idx < valid_ranges.size(); ++idx)
    {
      if (!valid_ranges[idx])
      {
        output_scan.ranges[idx] = std::numeric_limits<float>::quiet_NaN();
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
          RCLCPP_DEBUG_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
        if(parameter.get_name() == param_prefix_+"filter_type"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            filter_type = parameter.as_int();
        else if(parameter.get_name() == param_prefix_+"max_range" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            max_range = parameter.as_double();
        else if(parameter.get_name() == param_prefix_+"max_range_difference" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            max_range_difference = parameter.as_double();
        else if(parameter.get_name() == param_prefix_+"filter_window" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            filter_window = parameter.as_int();
        else
          if(logging_interface_ != nullptr) RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
      }

    switch (filter_type) {
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

  ////////////////////////////////////////////////////


private:
  WindowValidator* validator_;
  int filter_type = 0;
  double max_range = 0;
  double max_range_difference = 0;
  int filter_window = 0;
};
}
#endif /* speckle_filter.h */
