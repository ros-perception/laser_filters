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

#ifndef LASER_FILTERS__ARRAY_FILTER_HPP_
#define LASER_FILTERS__ARRAY_FILTER_HPP_

#include <map>
#include <iostream>
#include <sstream>

#include "boost/thread/mutex.hpp"
#include "boost/scoped_ptr.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "filters/median.hpp"
#include "filters/mean.hpp"
#include "filters/filter_chain.hpp"

namespace laser_filters
{

/** \brief A class to provide median filtering of laser scans in time*/
class LaserArrayFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  /** \brief Constructor
   * \param averaging_length How many scans to average over.
   */
  LaserArrayFilter();
  ~LaserArrayFilter();

  bool configure();

  /** \brief Update the filter and get the response
   * \param scan_in The new scan to filter
   * \param scan_out The filtered scan
   */
  bool update(const sensor_msgs::msg::LaserScan & scan_in, sensor_msgs::msg::LaserScan & scan_out);

private:
  unsigned int filter_length_;  // How many scans to average over
  unsigned int num_ranges_;  // How many data point are in each row

  rclcpp::Parameter range_config_;
  rclcpp::Parameter intensity_config_;

  boost::mutex data_lock;  // Protection from multi threaded programs
  sensor_msgs::msg::LaserScan temp_scan_;  /** \todo cache only shallow info not full scan */

  filters::MultiChannelFilterChain<float> * range_filter_;
  filters::MultiChannelFilterChain<float> * intensity_filter_;

  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");
};


}  // namespace laser_filters

#endif  // LASER_FILTERS__ARRAY_FILTER_HPP_
