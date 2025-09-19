/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Kevin Hallenbeck
*********************************************************************/
#ifndef LASER_SCAN_ANGULAR_BOUNDS_FILTER_IN_PLACE_H
#define LASER_SCAN_ANGULAR_BOUNDS_FILTER_IN_PLACE_H

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_filters
{
  class LaserScanAngularBoundsFilterInPlace : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
    public:
      double lower_angle_;
      double upper_angle_;
      bool replace_with_nan_;
      bool enabled_;

      bool configure()
      {
        lower_angle_ = 0;
        upper_angle_ = 0;

        if (!getParam("lower_angle", lower_angle_, false) || !getParam("upper_angle", upper_angle_, false))
        {
          RCLCPP_ERROR(logging_interface_->get_logger(), "Both the lower_angle and upper_angle parameters must be set to use this filter.");
          return false;
        }

        //toggle to use NaN for filtering scans; defaults to false for backward compatibility.
        //https://github.com/ros-perception/laser_filters/pull/202
        replace_with_nan_ = false;
        getParam("replace_with_nan", replace_with_nan_, false);

        getParam("enabled", enabled_, false, true);

        return true;
      }

      virtual ~LaserScanAngularBoundsFilterInPlace(){}

      bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan){
        filtered_scan = input_scan; //copy entire message

        if (!enabled_) {
          return true;
        }

        double current_angle = input_scan.angle_min;
        unsigned int count = 0;
        float replace_value = replace_with_nan_ ? std::numeric_limits<float>::quiet_NaN() : input_scan.range_max + 1.0;
        //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
          if((current_angle > lower_angle_) && (current_angle < upper_angle_)){
            filtered_scan.ranges[i] = replace_value;
            if(i < filtered_scan.intensities.size()){
              filtered_scan.intensities[i] = 0.0;
            }
            count++;
          }
          current_angle += input_scan.angle_increment;
        }

        RCLCPP_DEBUG(logging_interface_->get_logger(), "Filtered out %u points from the laser scan.", count);

        return true;

      }

      rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters)
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (auto parameter : parameters)
        {
          std::string full_name = parameter.get_name();
          std::string param_name = full_name.substr(full_name.find_last_of('.') + 1);

          if(param_name == "enabled" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            enabled_ = parameter.as_bool();
          else if(param_name == "lower_angle" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            lower_angle_ = parameter.as_double();
          else if(param_name == "upper_angle" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            upper_angle_ = parameter.as_double();
          else if(param_name == "replace_with_nan" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            replace_with_nan_ = parameter.as_bool();
        }

        return result;
      };
    };
};
#endif
