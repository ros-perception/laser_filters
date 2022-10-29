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
#include <ros/node_handle.h>

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
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::SpeckleFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<laser_filters::SpeckleFilterConfig>::CallbackType f;
  f = [this](auto& config, auto level){ reconfigureCB(config, level); };
  dyn_server_->setCallback(f);

  getParam("filter_type", config_.filter_type);
  getParam("max_range", config_.max_range);
  getParam("max_range_difference", config_.max_range_difference);
  getParam("filter_window", config_.filter_window);
  dyn_server_->updateConfig(config_);
  return true;
}

bool LaserScanSpeckleFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan)
{
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  output_scan = input_scan;

  std::vector<bool> &valid_ranges = valid_ranges_work_;

  /*Check if range size is big enough to use the filter window */
  if (output_scan.ranges.size() <= config_.filter_window + 1)
  {
    ROS_ERROR("Scan ranges size is too small: size = %ld", output_scan.ranges.size());
    return false;
  }

  size_t i = 0;
  size_t i_max = input_scan.ranges.size();
  valid_ranges.clear();
  while (i < i_max) {
    bool out_of_range = output_scan.ranges[i] > config_.max_range;
    valid_ranges.push_back(out_of_range);
    ++i;
  }

  i = 0;
  i_max = input_scan.ranges.size() - config_.filter_window + 1;
  while (i < i_max) {
    bool window_valid = validator_->checkWindowValid(
      output_scan, i, config_.filter_window, config_.max_range_difference
    );
    if (window_valid) {
      size_t j = i, j_max = i + config_.filter_window;
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

  return true;
}

void LaserScanSpeckleFilter::reconfigureCB(laser_filters::SpeckleFilterConfig& config, uint32_t level)
{
  config_ = config;

  switch (config_.filter_type) {
    case laser_filters::SpeckleFilter_RadiusOutlier:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::RadiusOutlierWindowValidator();
      break;

    case laser_filters::SpeckleFilter_Distance:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new laser_filters::DistanceWindowValidator();
      break;

    default:
      break;
  }

}
}
