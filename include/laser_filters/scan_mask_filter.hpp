/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, laser_filters authors
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

#ifndef LASER_FILTERS__SCAN_MASK_FILTER_HPP_
#define LASER_FILTERS__SCAN_MASK_FILTER_HPP_
/**
\author Atsushi Watanabe (SEQSENSE, Inc.)
\brief LaserScanMaskFilter removes points on directions defined in a mask from a laser scan.
**/

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "filters/filter_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace laser_filters
{

class LaserScanMaskFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
private:
  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");

public:
  std::map<std::string, std::vector<int64_t>> masks_;

  LaserScanMaskFilter()
  {
  }

  virtual ~LaserScanMaskFilter()
  {
  }

  bool configure()
  {
    rclcpp::Parameter config;

    // Get the parameter value.
    if (!node_->get_parameter("params.masks.laser", config)) {
      RCLCPP_ERROR(laser_filters_logger,
        "LaserScanMaskFilter: masks is not defined in the config.");
      return false;
    }

    if (config.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      RCLCPP_INFO(laser_filters_logger, "LaserScanMaskFilter: Name: %s - Type: %d",
        config.get_name().c_str(), config.get_type());
      masks_["laser"] = config.as_integer_array();
    } else {
      RCLCPP_ERROR(laser_filters_logger,
        "LaserScanMaskFilter: masks must be an array of frame_ids with direction list.");
      return false;
    }

#ifdef ROS2_SUPPORTS_PARAMETER_ARRAYS
    // TODO(Rohit): support parameter array

    if (config.get_type() == rclcpp::parameter::ParameterType::TypeArray) {
      RCLCPP_ERROR(laser_filters_logger,
        "LaserScanMaskFilter: masks must be an array of frame_ids with direction list.");
      return false;
    }
    for (rclcpp::parameter::ParameterIterator it = config.begin();
      it != config.end(); ++it)
    {
      if (it->second.get_type() == rclcpp::parameter::ParameterType::TypeArray) {
        std::string frame_id = (std::string)(it->first);
        masks_[frame_id] = std::vector<size_t>();
        try {
          for (size_t i = 0; i < it->second.size(); ++i) {
            size_t id = static_cast<int>(it->second[i]);
            masks_[frame_id].push_back(id);
          }
          RCLCPP_INFO(laser_filters_logger,
            "LaserScanMaskFilter: %s: %d directions will be masked.",
            frame_id.c_str(), (int)masks_[frame_id].size());
        } catch (rclcpp::parameter::ParameterException & e) {
          RCLCPP_ERROR(laser_filters_logger, "LaserScanMaskFilter: %s", e.get_message().c_str());
          return false;
        }
      }
    }
#endif  // ROS2_SUPPORTS_PARAMETER_ARRAYS

    return true;
  }

  bool update(const sensor_msgs::msg::LaserScan & data_in, sensor_msgs::msg::LaserScan & data_out)
  {
    data_out = data_in;
    if (masks_.find(data_out.header.frame_id) == masks_.end()) {
      RCLCPP_WARN(laser_filters_logger, "LaserScanMaskFilter: frame_id %s is not registered.",
        data_out.header.frame_id.c_str());
      return true;
    }
    const std::vector<int64_t> & mask = masks_[data_out.header.frame_id];
    const size_t len = data_out.ranges.size();

    for (std::vector<int64_t>::const_iterator it = mask.begin();
      it != mask.end(); ++it)
    {
      if (*it > len) {continue;}
      data_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();
    }

    return true;
  }
};

}  // namespace laser_filters

#endif  // LASER_FILTERS__SCAN_MASK_FILTER_HPP_
