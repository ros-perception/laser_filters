/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2008-2021 Radu Bogdan Rusu <rusu@cs.tum.edu> and other laser_filters authors
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <laser_filters/scan_shadows_filter.h>

#include <angles/angles.h>

namespace laser_filters
{

ScanShadowsFilter::ScanShadowsFilter()
{
}

ScanShadowsFilter::~ScanShadowsFilter()
{
}
    
bool ScanShadowsFilter::configure()
{
  node_ = std::make_shared<rclcpp::Node>(getName());
  // dynamic reconfigure parameters callback:
  on_set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&ScanShadowsFilter::reconfigureCB, this, std::placeholders::_1));


  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_angle"), min_angle_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
    return false;
  }
  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_angle"), max_angle_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
    return false;
  }
  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("window"), window_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given window.\n");
    return false;
  }
  neighbors_ = 0;  // default value
  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("neighbors"), neighbors_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given neighbors.\n");
  }


  remove_shadow_start_point_ = false;  // default value
  filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("remove_shadow_start_point"), remove_shadow_start_point_);
  RCLCPP_INFO(logging_interface_->get_logger(), "Remove shadow start point: %s", remove_shadow_start_point_ ? "true" : "false");

  if (min_angle_ < 0)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "min_angle must be 0 <= min_angle. Forcing min_angle = 0.\n");
    min_angle_ = 0.0;
  }
  if (90 < min_angle_)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "min_angle must be min_angle <= 90. Forcing min_angle = 90.\n");
    min_angle_ = 90.0;
  }
  if (max_angle_ < 90)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "max_angle must be 90 <= max_angle. Forcing max_angle = 90.\n");
    max_angle_ = 90.0;
  }
  if (180 < max_angle_)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "max_angle must be max_angle <= 180. Forcing max_angle = 180.\n");
    max_angle_ = 180.0;
  }

  shadow_detector_.configure(
      angles::from_degrees(min_angle_),
      angles::from_degrees(max_angle_));

  angle_increment_ = 0;
  return true;
}

rcl_interfaces::msg::SetParametersResult ScanShadowsFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
    boost::recursive_mutex::scoped_lock lock(own_mutex_);

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == "min_angle"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          min_angle_ = parameter.as_double();
      else if(parameter.get_name() == "max_angle" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_angle_ = parameter.as_double();
      else if(parameter.get_name() == "neighbors" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          neighbors_ = parameter.as_int();
      else if(parameter.get_name() == "window" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          window_ = parameter.as_int();
      else if(parameter.get_name() == "remove_shadow_start_point" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          remove_shadow_start_point_ = parameter.as_bool();
      else
        RCLCPP_WARN(node_->get_logger(), "Unknown parameter");
    }
    shadow_detector_.configure(
        angles::from_degrees(min_angle_),
        angles::from_degrees(max_angle_));
    angle_increment_ = 0;

  return result;
}

bool ScanShadowsFilter::update(const sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::LaserScan& scan_out)
{
    auto start = std::chrono::high_resolution_clock::now();

    boost::recursive_mutex::scoped_lock lock(own_mutex_);

    // copy across all data first
    scan_out = scan_in;

    int size = scan_in.ranges.size();
    int max_y;
    int max_neighbors;
    prepareForInput(scan_in.angle_increment);
    // For each point in the current line scan
    for (int i = 0; i < size; i++)
    {
      max_y = std::min<int>(size - i, window_ + 1);
      for (int y = std::max<int>(-i, -window_); y < max_y; y++)
      {
        if (y == 0)
        {
          continue;
        }

        if (shadow_detector_.isShadow(
                scan_in.ranges[i], scan_in.ranges[i + y], sin_map_[y + window_], cos_map_[y + window_]))
        {
          max_neighbors = std::min<int>(i + neighbors_, size - 1);
          for (int index = std::max<int>(i - neighbors_, 0); index <= max_neighbors; index++)
          {
            if (scan_in.ranges[i] < scan_in.ranges[index])
            {  // delete neighbor if they are farther away (note not self)
              scan_out.ranges[index] = std::numeric_limits<float>::quiet_NaN(); 
            }
          }
          if (remove_shadow_start_point_)
          {
              scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN(); 
          }
          break;
        }
      }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto update_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_DEBUG(logging_interface_->get_logger(), "LaserScanShadowsFilter update took %lu microseconds", update_elapsed);

    return true;
}

void ScanShadowsFilter::prepareForInput(const float angle_increment) {
  if (angle_increment_ != angle_increment) {
    RCLCPP_DEBUG(logging_interface_->get_logger(), "[ScanShadowsFilter] No precomputed map given. Computing one.");
    angle_increment_ = angle_increment;
    sin_map_.clear();
    cos_map_.clear();

    float included_angle = -window_ * angle_increment;
    for (int i = -window_; i <= window_; ++i) {
      sin_map_.push_back(fabs(sinf(included_angle)));
      cos_map_.push_back(cosf(included_angle));
      included_angle += angle_increment;
    }
  }
}
}
