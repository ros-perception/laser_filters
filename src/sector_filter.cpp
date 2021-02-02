/*********************************************************************
* BSD 2-Clause License
*
* Copyright (c) 2021, Jimmy F. Klarke
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author: Jimmy F. Klarke
*********************************************************************/

#include <math.h>

#include <laser_filters/sector_filter.h>
#include <ros/node_handle.h>

namespace laser_filters
{

LaserScanSectorFilter::LaserScanSectorFilter()
{
}

bool LaserScanSectorFilter::configure()
{
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<SectorFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<SectorFilterConfig>::CallbackType f;
  f = boost::bind(&LaserScanSectorFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  getParam("angle_min", config_.angle_min);
  getParam("angle_max", config_.angle_max);
  getParam("range_min", config_.range_min);
  getParam("range_max", config_.range_max);
  getParam("clear_inside", config_.clear_inside);
  getParam("invert", config_.invert);

  ROS_INFO("clear_inside(!invert): %s", (isClearInside() ? "true" : "false"));

  dyn_server_->updateConfig(config_);
  return true;
}

void LaserScanSectorFilter::reconfigureCB(SectorFilterConfig& config, uint32_t level)
{
  config_ = config;
}

bool LaserScanSectorFilter::isClearInside()
{
  bool clear_inside = config_.clear_inside;
  bool invert = config_.invert;

  clear_inside = invert ? false : clear_inside;

  return clear_inside;
}

bool LaserScanSectorFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
{
  filtered_scan = input_scan; //copy entire message

  double angle_min = config_.angle_min;
  double angle_max = config_.angle_max;
  double range_min = config_.range_min;
  double range_max = config_.range_max;
  bool clear_inside = isClearInside();

  double angle_delta = angle_max - angle_min;
  if (angle_max < angle_min)
  {
    angle_delta += M_PI * 2;
  }

  double current_angle = input_scan.angle_min;
  unsigned int count = 0;
  //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
  for (size_t i = 0; i < input_scan.ranges.size(); ++i)
  {
    current_angle = (i == 0) ? current_angle : (current_angle + input_scan.angle_increment);

    double current_range = input_scan.ranges[i];
    double current_angle_delta = current_angle - angle_min;
    if ((angle_max < angle_min) && (current_angle_delta < 0))
    {
      current_angle_delta += M_PI * 2;
    }

    if (clear_inside != ((current_angle_delta > 0)
                      && (current_angle_delta < angle_delta)
                      && (current_range > range_min)
                      && (current_range < range_max)))
    {
      continue;
    }

    filtered_scan.ranges[i] = input_scan.range_max + 1.0;
    if (i < filtered_scan.intensities.size())
    {
      filtered_scan.intensities[i] = 0.0;
    }
    count++;
  }

  ROS_DEBUG("Filtered out %u points from the laser scan.", count);

  return true;
}

} // end namespace laser_filters
