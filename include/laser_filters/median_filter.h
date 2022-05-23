/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#ifndef LASER_SCAN_MEDIAN_FILTER_H
#define LASER_SCAN_MEDIAN_FILTER_H

#include <map>
#include <iostream>
#include <sstream>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "filters/median.hpp"
#include "filters/mean.hpp"
#include "filters/filter_chain.hpp"

namespace laser_filters
{

  /** \brief A class to provide median filtering of laser scans in time*/
  class LaserMedianFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
  {
  public:
    /** \brief Constructor
   * \param averaging_length How many scans to average over.
   */
    LaserMedianFilter()
        : num_ranges_(1), /*parameter_value_(),*/ range_filter_(NULL), intensity_filter_(NULL)
    {
      RCLCPP_WARN(logging_interface_->get_logger(), "LaserMedianFilter has been deprecated.  Please use LaserArrayFilter instead.\n");
    };
    ~LaserMedianFilter()
    {
      delete range_filter_;
      delete intensity_filter_;
    };

    bool configure()
    {

      // if (!getParam(, parameter_value_))
      // {
      //   RCLCPP_ERROR(get_logger(), "Cannot Configure LaserMedianFilter: Didn't find \"internal_filter\" tag within LaserMedianFilter params. Filter definitions needed inside for processing range and intensity");
      //   return false;
      // }

      if (range_filter_)
        delete range_filter_;
      range_filter_ = new filters::MultiChannelFilterChain<float>("float");
      if (!range_filter_->configure(num_ranges_, "internal_filter", logging_interface_, params_interface_))
        return false;

      if (intensity_filter_)
        delete intensity_filter_;
      intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
      if (!intensity_filter_->configure(num_ranges_, "internal_filter", logging_interface_, params_interface_))
        return false;
      return true;
    };

    /** \brief Update the filter and get the response
   * \param scan_in The new scan to filter
   * \param scan_out The filtered scan
   */
    bool update(const sensor_msgs::msg::LaserScan &scan_in, sensor_msgs::msg::LaserScan &scan_out)
    {
      if (!this->configured_)
      {
        RCLCPP_ERROR(logging_interface_->get_logger(), "LaserMedianFilter not configured");
        return false;
      }
      std::lock_guard<std::mutex> lock(data_lock);
      scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too

      if (scan_in.ranges.size() != num_ranges_) //Reallocating
      {
        RCLCPP_INFO(logging_interface_->get_logger(), "Laser filter clearning and reallocating due to larger scan size");
        delete range_filter_;
        delete intensity_filter_;

        num_ranges_ = scan_in.ranges.size();

        range_filter_ = new filters::MultiChannelFilterChain<float>("float");
        if (!range_filter_->configure(num_ranges_, "internal_filter", logging_interface_, params_interface_))
          return false;

        intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
        if (!intensity_filter_->configure(num_ranges_, "internal_filter", logging_interface_, params_interface_))
          return false;
      }

      /** \todo check for length of intensities too */
      range_filter_->update(scan_in.ranges, scan_out.ranges);
      intensity_filter_->update(scan_in.intensities, scan_out.intensities);

      return true;
    }

  private:
    unsigned int filter_length_; ///How many scans to average over
    unsigned int num_ranges_;    /// How many data point are in each row

    std::mutex data_lock;                 /// Protection from multi threaded programs
    sensor_msgs::msg::LaserScan temp_scan_; /** \todo cache only shallow info not full scan */

    // rclcpp::parameter::ParameterVariant parameter_value_;

    filters::MultiChannelFilterChain<float> *range_filter_;
    filters::MultiChannelFilterChain<float> *intensity_filter_;
  };

} // namespace laser_filters

#endif //LASER_SCAN_UTILS_LASERSCAN_H
