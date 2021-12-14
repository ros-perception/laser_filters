/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu> and other laser_filters authors
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
 *********************************************************************/

/*
  \author Radu Bogdan Rusu <rusu@cs.tum.edu> Tully Foote <tfoote@willowgarage.com>
*/

#ifndef LASER_SCAN_SHADOWS_FILTER_H
#define LASER_SCAN_SHADOWS_FILTER_H

#include "laser_filters/scan_shadow_detector.h"

#include <filters/filter_base.hpp>
#include <sensor_msgs/LaserScan.h>
#include <laser_filters/ScanShadowsFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace laser_filters
{
/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line 
 */

class ScanShadowsFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  double laser_max_range_;        // Used in laser scan projection
  double min_angle_, max_angle_;  // Filter angle threshold
  int window_, neighbors_;
  bool remove_shadow_start_point_;  // Whether to also remove the start point of the shadow

  ScanShadowDetector shadow_detector_;

  std::shared_ptr<dynamic_reconfigure::Server<laser_filters::ScanShadowsFilterConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;
  ScanShadowsFilterConfig param_config;

  ScanShadowsFilter();
  virtual ~ScanShadowsFilter();

  /**@b Configure the filter from XML */
  bool configure();

  void reconfigureCB(ScanShadowsFilterConfig& config, uint32_t level);

  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);
private:
  float angle_increment_;
  std::vector<float> sin_map_;
  std::vector<float> cos_map_;
  
  void prepareForInput(const float angle_increment);
};
}

#endif  // LASER_SCAN_SHADOWS_FILTER_H
