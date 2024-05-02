/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu> and other laser_filters authors
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

/*
  \author Radu Bogdan Rusu <rusu@cs.tum.edu> Tully Foote <tfoote@willowgarage.com>
*/

#ifndef LASER_SCAN_SHADOWS_FILTER_H
#define LASER_SCAN_SHADOWS_FILTER_H

#include "laser_filters/scan_shadow_detector.h"

#include "filters/filter_base.hpp"
#include "laser_filters/scan_shadow_detector.h"
#include <sensor_msgs/msg/laser_scan.hpp>


#ifdef _WIN32
#define _USE_MATH_DEFINES // for C  
#include <math.h>  
#endif // _WIN32

#include <angles/angles.h>
#include <boost/thread.hpp>

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

  boost::recursive_mutex own_mutex_;
  ////////////////////////////////////////////////////////////////////////////////
  ScanShadowsFilter()
  {
  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_angle"), min_angle_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_angle"), max_angle_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("window"), window_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: ShadowsFilter was not given window.\n");
      return false;
    }
    neighbors_ = 0;  // default value
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("neighbors"), neighbors_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: ShadowsFilter was not given neighbors.\n");
    }

    remove_shadow_start_point_ = false;  // default value
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("remove_shadow_start_point"), remove_shadow_start_point_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: ShadowsFilter was not given remove_shadow_start_point.\n");
    }

    if (min_angle_ < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "min_angle must be 0 <= min_angle. Forcing min_angle = 0.\n");
      min_angle_ = 0.0;
    }
    if (90 < min_angle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "min_angle must be min_angle <= 90. Forcing min_angle = 90.\n");
      min_angle_ = 90.0;
    }
    if (max_angle_ < 90)
    {
      RCLCPP_ERROR(node_->get_logger(), "max_angle must be 90 <= max_angle. Forcing max_angle = 90.\n");
      max_angle_ = 90.0;
    }
    if (180 < max_angle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "max_angle must be max_angle <= 180. Forcing max_angle = 180.\n");
      max_angle_ = 180.0;
    }
    shadow_detector_.configure(
        angles::from_degrees(min_angle_),
        angles::from_degrees(max_angle_));
    angle_increment_=0;

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
    return true;
  }
  
  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters)
  {
    boost::recursive_mutex::scoped_lock lock(own_mutex_);

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
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
    }
    shadow_detector_.configure(
        angles::from_degrees(min_angle_),
        angles::from_degrees(max_angle_));

    angle_increment_=0;
    return result;
  }

private:
  float angle_increment_;
  std::vector<float> sin_map_;
  std::vector<float> cos_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  void prepareForInput(const float angle_increment) {
    if (angle_increment_ != angle_increment) {
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

  ////////////////////////////////////////////////////////////////////////////////
};
}

#endif  // LASER_SCAN_SHADOWS_FILTER_H
