/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
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
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
  \author Radu Bogdan Rusu <rusu@cs.tum.edu> Tully Foote <tfoote@willowgarage.com>


*/

#ifndef LASER_SCAN_SHADOWS_FILTER_H
#define LASER_SCAN_SHADOWS_FILTER_H

#include <set>

#include "filters/filter_base.hpp"
#include "laser_filters/scan_shadow_detector.h"
#include <sensor_msgs/msg/laser_scan.hpp>

#ifdef _WIN32
#define _USE_MATH_DEFINES // for C  
#include <math.h>  
#endif // _WIN32

#include <angles/angles.h>

namespace laser_filters
{
/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line 
 */

class ScanShadowsFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  double laser_max_range_;        // Used in laser scan projection
  double min_angle_, max_angle_;  // Filter angle threshold
  int window_, neighbors_;
  bool remove_shadow_start_point_;

  ScanShadowDetector shadow_detector_;

  ////////////////////////////////////////////////////////////////////////////////
  ScanShadowsFilter()
  {
  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_angle"), min_angle_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_angle"), max_angle_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("window"), window_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given window.\n");
      return false;
    }
    neighbors_ = 0;  // default value
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("neighbors"), neighbors_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given neighbors.\n");
    }
    remove_shadow_start_point_ = false;  // default value
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("remove_shadow_start_point"), remove_shadow_start_point_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: ShadowsFilter was not given remove_shadow_start_point.\n");
    }

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

    RCLCPP_INFO(logging_interface_->get_logger(), "In shadow configure done");

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ScanShadowsFilter()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::LaserScan& scan_out)
  {
    // copy across all data first
    scan_out = scan_in;

    std::set<int> indices_to_delete;
    // For each point in the current line scan
    for (unsigned int i = 0; i < scan_in.ranges.size(); i++)
    {
      for (int y = -window_; y < window_ + 1; y++)
      {
        int j = i + y;
        if (j < 0 || j >= (int)scan_in.ranges.size() || (int)i == j)
        {  // Out of scan bounds or itself
          continue;
        }

        if (shadow_detector_.isShadow(
                scan_in.ranges[i], scan_in.ranges[j], y * scan_in.angle_increment))
        {
          for (int index = std::max<int>(i - neighbors_, 0); index <= std::min<int>(i + neighbors_, (int)scan_in.ranges.size() - 1); index++)
          {
            if (scan_in.ranges[i] < scan_in.ranges[index])
            {  // delete neighbor if they are farther away (note not self)
              indices_to_delete.insert(index);
            }
          }
          if (remove_shadow_start_point_)
          {
              scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN(); 
          }
        }
      }
    }

    if(logging_interface_!= nullptr) 
    {
      RCLCPP_DEBUG(logging_interface_->get_logger(), "ScanShadowsFilter removing %d Points from scan with min angle: %.2f, max angle: %.2f, neighbors: %d, and window: %d",
                  (int)indices_to_delete.size(), min_angle_, max_angle_, neighbors_, window_);
    } 
    for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
    {
      scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();  // Failed test to set the ranges to invalid value
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
      if(parameter.get_name() == param_prefix_+"min_angle"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          min_angle_ = parameter.as_double();
      else if(parameter.get_name() == param_prefix_+"max_angle" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_angle_ = parameter.as_double();
      else if(parameter.get_name() == param_prefix_+"neighbors" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          neighbors_ = parameter.as_int();
      else if(parameter.get_name() == param_prefix_+"window" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          window_ = parameter.as_int();
      else if(parameter.get_name() == param_prefix_+"remove_shadow_start_point" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          remove_shadow_start_point_ = parameter.as_bool(); 
    }
    shadow_detector_.configure(
        angles::from_degrees(min_angle_),
        angles::from_degrees(max_angle_));

    return result;
  }
  ////////////////////////////////////////////////////////////////////////////////
};
}

#endif  // LASER_SCAN_SHADOWS_FILTER_H
