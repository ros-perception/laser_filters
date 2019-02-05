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

#ifndef LASER_FILTERS__SCAN_SHADOWS_FILTER_HPP_
#define LASER_FILTERS__SCAN_SHADOWS_FILTER_HPP_

#include <set>
#include <algorithm>
#include <limits>

#include "filters/filter_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#ifdef _WIN32
#define _USE_MATH_DEFINES  // for C
#include <math.h>
#endif  // _WIN32

#include <angles/angles.h>

namespace laser_filters
{

/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line
 */

class ScanShadowsFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
private:
  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");

public:
  double laser_max_range_;           // Used in laser scan projection
  double min_angle_, max_angle_;          // Filter angle threshold
  int window_, neighbors_;

  ScanShadowsFilter()
  {
  }

  /**@b Configure the filter */
  bool configure()
  {
    // Get the parameter value.
    if (!FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("params.min_angle",
      min_angle_))
    {
      RCLCPP_ERROR(laser_filters_logger, "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("params.max_angle",
      max_angle_))
    {
      RCLCPP_ERROR(laser_filters_logger, "Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("params.window", window_)) {
      RCLCPP_ERROR(laser_filters_logger, "Error: ShadowsFilter was not given window.\n");
      return false;
    }
    // default value i.e. "neighbors_ = 0"
    if (!FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter_or("params.neighbors",
      neighbors_, 0))
    {
      RCLCPP_INFO(laser_filters_logger, "Error: ShadowsFilter was not given neighbors.\n");
    }

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ScanShadowsFilter() {}

  /** @brief calculate the perpendicular angle at the end of r1 to get to r2
   * See http://en.wikipedia.org/wiki/Law_of_cosines */
  inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
  {
    return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
  }


  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::msg::LaserScan & scan_in, sensor_msgs::msg::LaserScan & scan_out)
  {
    // copy across all data first
    scan_out = scan_in;

    std::set<int> indices_to_delete;
    // For each point in the current line scan
    for (unsigned int i = 0; i < scan_in.ranges.size(); i++) {
      for (int y = -window_; y < window_ + 1; y++) {
        int j = i + y;
        if (j < 0 || j >= static_cast<int>(scan_in.ranges.size()) || static_cast<int>(i) == j) {
          // Out of scan bounds or itself
          continue;
        }

        double angle =
          abs(angles::to_degrees(getAngleWithViewpoint(scan_in.ranges[i], scan_in.ranges[j],
            y * scan_in.angle_increment)));
        if (angle < min_angle_ || angle > max_angle_) {
          for (int index = std::max<int>(i - neighbors_, 0);
            index <= std::min<int>(i + neighbors_, static_cast<int>(scan_in.ranges.size()) - 1);
            index++)
          {
            if (scan_in.ranges[i] < scan_in.ranges[index]) {
              // delete neighbor if they are farther away (note not self)
              indices_to_delete.insert(index);
            }
          }
        }
      }
    }

    RCLCPP_DEBUG(laser_filters_logger,
      "ScanShadowsFilter removing %d Points from scan with: ", (int)indices_to_delete.size());
    RCLCPP_DEBUG(laser_filters_logger,
      "min angle: %.2f, max angle: %.2f, neighbors: %d, and window: %d",
      min_angle_, max_angle_, neighbors_, window_);
    for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end();
      ++it)
    {
      scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();
      // Failed test to set the ranges to invalid value
    }
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
};
}  // namespace laser_filters

#endif  // LASER_FILTERS__SCAN_SHADOWS_FILTER_HPP_
