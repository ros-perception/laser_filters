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

#ifndef LASER_SCAN_SECTOR_FILTER_IN_PLACE_H
#define LASER_SCAN_SECTOR_FILTER_IN_PLACE_H

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 

namespace laser_filters
{

class LaserScanSectorFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanSectorFilter(){}

  bool configure()
  {
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("angle_min"), angle_min_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given angle_min.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("angle_max"), angle_max_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given angle_max.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("range_min"), range_min_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given range_min.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("range_max"), range_max_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given range_max.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("clear_inside"), clear_inside_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given clear_inside.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("invert"), invert_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: LaserScanSectorFilter was not given invert.\n");
      return false;
    }

    RCLCPP_DEBUG(logging_interface_->get_logger(), "clear_inside(!invert): %s", (isClearInside() ? "true" : "false"));
    return true;
  }

  bool isClearInside()
  {
     return invert_ ? false : clear_inside_;
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan)
  {
    output_scan = input_scan; //copy entire message
    bool clear_inside = isClearInside();

    double angle_delta = angle_max_ - angle_min_;
    if (angle_max_ < angle_min_)
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
      double current_angle_delta = current_angle - angle_min_;
      if ((angle_max_ < angle_min_) && (current_angle_delta < 0))
      {
        current_angle_delta += M_PI * 2;
      }

      if (clear_inside != ((current_angle_delta > 0)
                        && (current_angle_delta < angle_delta)
                        && (current_range > range_min_)
                        && (current_range < range_max_)))
      {
        continue;
      }

      output_scan.ranges[i] = input_scan.range_max + 1.0;
      if (i < output_scan.intensities.size())
      {
        output_scan.intensities[i] = 0.0;
      }
      count++;
    }

    RCLCPP_DEBUG(logging_interface_->get_logger(), "Filtered out %u points from the laser scan.", count);

    return true;
  }

  virtual ~LaserScanSectorFilter(){}

private:
  double angle_min_;
  double angle_max_;
  double range_min_;
  double range_max_;
  bool clear_inside_;
  bool invert_;
};

} // end namespace laser_filters

#endif // LASER_SCAN_SECTOR_FILTER_IN_PLACE_H
