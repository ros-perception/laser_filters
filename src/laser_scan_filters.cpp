/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "laser_filters/median_filter.h"
#include "laser_filters/array_filter.h"
#include "laser_filters/intensity_filter.h"
#include "laser_filters/range_filter.h"
#include "laser_filters/scan_shadows_filter.h"
#include "laser_filters/footprint_filter.h"
#include "laser_filters/interpolation_filter.h"
#include "laser_filters/angular_bounds_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"


PLUGINLIB_DECLARE_CLASS(laser_filters, LaserMedianFilter, laser_filters::LaserMedianFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, LaserArrayFilter, laser_filters::LaserArrayFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, LaserScanIntensityFilter, laser_filters::LaserScanIntensityFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, LaserScanRangeFilter, laser_filters::LaserScanRangeFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, LaserScanAngularBoundsFilter, laser_filters::LaserScanAngularBoundsFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, LaserScanFootprintFilter, laser_filters::LaserScanFootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, ScanShadowsFilter, laser_filters::ScanShadowsFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(laser_filters, InterpolationFilter, laser_filters::InterpolationFilter, filters::FilterBase<sensor_msgs::LaserScan>)
