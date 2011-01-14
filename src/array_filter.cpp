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

#include "laser_filters/array_filter.h"

namespace laser_filters
{
LaserArrayFilter::LaserArrayFilter() :
  num_ranges_(1), range_filter_(NULL), intensity_filter_(NULL)
{
  
};

bool LaserArrayFilter::configure()
{
 
  bool found_range_config = getParam("range_filter_chain", range_config_);
  bool found_intensity_config = getParam("intensity_filter_chain", intensity_config_);
 
  if (!found_range_config && !found_intensity_config)
  {
    ROS_ERROR("Cannot Configure LaserArrayFilter: Didn't find \"range_filter\" or \"intensity _filter\" tag within LaserArrayFilter params. Filter definitions needed inside for processing range and intensity");
    return false;
  }
  
  if (range_filter_)
    delete range_filter_;

  if (intensity_filter_)
    delete intensity_filter_;
  
  if (found_range_config)
  {
    range_filter_ = new filters::MultiChannelFilterChain<float>("float");
    if (!range_filter_->configure(num_ranges_, range_config_))
      return false;
  }

  if (found_intensity_config)
  {
    intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
    if (!intensity_filter_->configure(num_ranges_, intensity_config_))
      return false;
  }
  
  return true;
};

LaserArrayFilter::~LaserArrayFilter()
{
  if (range_filter_)
    delete range_filter_;

  if (intensity_filter_)
    delete intensity_filter_;
};

bool LaserArrayFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
  if (!this->configured_) 
  {
    ROS_ERROR("LaserArrayFilter not configured");
    return false;
  }

  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too

  if (scan_in.ranges.size() != num_ranges_) //Reallocating
  {
    num_ranges_ = scan_in.ranges.size();

    ROS_INFO("LaserArrayFilter cleaning and reallocating due to larger scan size");
    
    configure();
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}
}
