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
 * $Id: $
 *
 */

#include <string>
#include <map>

#include "laser_filters/array_filter.hpp"

namespace laser_filters
{
LaserArrayFilter::LaserArrayFilter()
: num_ranges_(1), range_filter_(NULL), intensity_filter_(NULL)
{
}

bool LaserArrayFilter::configure()
{
  if (range_filter_) {
    delete range_filter_;
  }

  if (intensity_filter_) {
    delete intensity_filter_;
  }

  rcl_interfaces::msg::ListParametersResult parameters_and_prefixes;
  std::stringstream ss1;
  std::map<std::string, std::string> filter_param;
  parameters_and_prefixes = node_->list_parameters({}, 10);
  for (auto & name : parameters_and_prefixes.names) {
    for (auto & parameter : node_->get_parameters({name})) {
      ss1 << "\nParameter name: " << parameter.get_name();
      ss1 << "\nParameter data_type: " << parameter.get_type();
      ss1 << "\nParameter value (" << parameter.get_type_name() <<
        "): " << parameter.value_to_string();
      filter_param[parameter.get_name()] = parameter.value_to_string();
    }
  }
  std::string p_name, param_name;
  for (std::map<std::string, std::string>::iterator filter_it = filter_param.begin();
    filter_it != filter_param.end(); ++filter_it)
  {
    std::string filter_name = filter_it->first;
    std::string filter_type = filter_it->second;
    if (std::string::npos != filter_name.find("type")) {
      if (std::string::npos != filter_name.find("intensity_filter_chain")) {
        std::string str2 = "intensity_filter_chain";
        std::size_t found = filter_name.find(str2);
        p_name = filter_name.substr(found + str2.length());
        int pos = filter_name.length() - p_name.length();
        param_name = filter_name.erase(pos, filter_name.length() + 1);
        RCLCPP_INFO(laser_filters_logger, "1. In LaserArrayFilter for intensity ");
        intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
        if (!intensity_filter_->configure(param_name, num_ranges_,
          FilterBase<sensor_msgs::msg::LaserScan>::node_))
        {
          RCLCPP_INFO(laser_filters_logger, "2. In LaserArrayFilter for intensity ");
          return false;
        }
      } else if (std::string::npos != filter_name.find("range_filter_chain")) {
        std::string str2 = "range_filter_chain";
        std::size_t found = filter_name.find(str2);
        p_name = filter_name.substr(found + str2.length());
        int pos = filter_name.length() - p_name.length();
        param_name = filter_name.erase(pos, filter_name.length() + 1);
        RCLCPP_INFO(laser_filters_logger, "1. In LaserArrayFilter for range ");
        range_filter_ = new filters::MultiChannelFilterChain<float>("float");
        if (!range_filter_->configure(param_name, num_ranges_,
          FilterBase<sensor_msgs::msg::LaserScan>::node_))
        {
          RCLCPP_INFO(laser_filters_logger, "2. In LaserArrayFilter for range ");
          return false;
        }
      }
    }
  }

  return true;
}

LaserArrayFilter::~LaserArrayFilter()
{
  if (range_filter_) {
    delete range_filter_;
  }

  if (intensity_filter_) {
    delete intensity_filter_;
  }
}

bool LaserArrayFilter::update(
  const sensor_msgs::msg::LaserScan & scan_in,
  sensor_msgs::msg::LaserScan & scan_out)
{
  if (!this->configured_) {
    RCLCPP_ERROR(laser_filters_logger, "LaserArrayFilter not configured");
    return false;
  }

  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in;  // Quickly pass through all data \todo don't copy data too

  if (scan_in.ranges.size() != num_ranges_) {  // Reallocating
    num_ranges_ = scan_in.ranges.size();

    RCLCPP_INFO(laser_filters_logger,
      "LaserArrayFilter cleaning and reallocating due to larger scan size");

    configure();
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}
}  // namespace laser_filters
