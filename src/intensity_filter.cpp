/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2020, Eurotec B.V.
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
*
*  \author Vijay Pradeep, Rein Appeldoorn
*
*********************************************************************/

#include <laser_filters/intensity_filter.h>
#include <ros/node_handle.h>

namespace laser_filters
{
LaserScanIntensityFilter::LaserScanIntensityFilter()
{
}

bool LaserScanIntensityFilter::configure()
{
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<IntensityFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<IntensityFilterConfig>::CallbackType f;
  f = boost::bind(&LaserScanIntensityFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  getParam("lower_threshold", config_.lower_threshold);
  getParam("upper_threshold", config_.upper_threshold);
  getParam("invert", config_.invert);

  getParam("filter_override_range", config_.filter_override_range);
  getParam("filter_override_intensity", config_.filter_override_intensity);
  dyn_server_->updateConfig(config_);
  return true;
}

bool LaserScanIntensityFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
{
  filtered_scan = input_scan;

  // Need to check ever reading in the current scan
  for (unsigned int i=0; i < input_scan.ranges.size() && i < input_scan.intensities.size(); i++)
  {
    float& range = filtered_scan.ranges[i];
    float& intensity = filtered_scan.intensities[i];

    // Is this reading below our lower threshold?
    // Is this reading above our upper threshold?
    bool filter = intensity <= config_.lower_threshold || intensity >= config_.upper_threshold;
    if (config_.invert)
    {
      filter = !filter;
    }

    if (filter)
    {
      if (config_.filter_override_range)
      {
        // If so, then make it an invalid value (NaN)
        range = std::numeric_limits<float>::quiet_NaN();
      }
      if (config_.filter_override_intensity)
      {
        intensity = 0.0;  // Not intense
      }
    }
    else
    {
      if (config_.filter_override_intensity)
      {
        intensity = 1.0;  // Intense
      }
    }
  }

  return true;
}

void LaserScanIntensityFilter::reconfigureCB(IntensityFilterConfig& config, uint32_t level)
{
  config_ = config;
}
}
