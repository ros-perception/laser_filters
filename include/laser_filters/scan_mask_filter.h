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

#ifndef LASER_FILTERS_SCAN_MASK_FILTER_H
#define LASER_FILTERS_SCAN_MASK_FILTER_H
/**
\author Atsushi Watanabe (SEQSENSE, Inc.)
\brief LaserScanMaskFilter removes points on directions defined in a mask from a laser scan.
**/

#include "filters/filter_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <limits>
#include <map>
#include <string>
#include <vector>

namespace laser_filters
{

class LaserScanMaskFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  std::map<std::string, std::vector<size_t> > masks_;

  bool configure()
  {
    std::string key_masks = param_prefix_ + "masks";
    for (auto i : params_interface_->get_parameter_overrides())
    {
      if (i.first.find(key_masks) == 0)
      {
        auto frame_id = i.first.substr(key_masks.size() + 1);
        std::vector<double> values;
        getParam("masks." + frame_id, values);
        masks_[frame_id].clear();
        for (size_t i = 0; i < values.size(); ++i)
        {
          size_t id = static_cast<int>(values[i]);
          masks_[frame_id].push_back(id);
        }
        RCLCPP_INFO(
            logging_interface_->get_logger(),
            "LaserScanMaskFilter: %s: %d directions will be masked.",
            frame_id.c_str(), (int)masks_[frame_id].size());
      }
    }

    if (masks_.empty())
    {
      RCLCPP_ERROR(
          logging_interface_->get_logger(),
          "LaserScanMaskFilter: masks is not defined in the config.");
      return false;
    }

    return true;
  }

  virtual ~LaserScanMaskFilter()
  {
  }

  bool update(const sensor_msgs::msg::LaserScan& data_in, sensor_msgs::msg::LaserScan& data_out)
  {
    data_out = data_in;
    if (masks_.find(data_out.header.frame_id) == masks_.end())
    {
      RCLCPP_WARN(
          logging_interface_->get_logger(),
          "LaserScanMaskFilter: frame_id %s is not registered.",
          data_out.header.frame_id.c_str());
      return true;
    }

    const std::vector<size_t> &mask = masks_[data_out.header.frame_id];
    const size_t len = data_out.ranges.size();
    for (std::vector<size_t>::const_iterator it = mask.begin();
        it != mask.end(); ++it)
    {
      if (*it > len) continue;
      data_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();
    }

    return true;
  }
};

}  // namespace laser_filters

#endif  // LASER_FILTERS_SCAN_MASK_FILTER_H
