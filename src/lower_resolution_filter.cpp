/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  lower_resolution_filter.cpp
 *
 *  author: Francesc Folch Company <frafolcm@gmail.com>
 */
#include "laser_filters/lower_resolution_filter.h"
#include <ros/ros.h>

int reduce_factor_;

bool laser_filters::LowerResolutionFilter::configure() 
{

  if(!getParam("reduce_factor", reduce_factor_)){
    ROS_ERROR("Cannot configure LowerResolutionFilter: Didn't find 'reduce_factor' paramerer.");
    return false;
  }
  return true;
}

bool laser_filters::LowerResolutionFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) 
{
  if (!this->configured_) 
  {
    ROS_ERROR("LaserArrayFilter not configured");
    return false;
  }

  scan_out = scan_in;

  int original_length = (scan_in.angle_max - scan_in.angle_min + scan_in.angle_increment) / scan_in.angle_increment;
  int reduced_length = std::ceil( original_length / reduce_factor_);

  // as we round up when defining reduced length, we have to take in count the last element of the array 
  // if the reduce factor is divisible by original length, we pick the nth last element (n=reduce_factor)
  // else, nth last element will be defined by the mod operation
  int mod_angles = original_length % reduce_factor_;
  float angle_max_out;
  int last_element;
  if(mod_angles != 0) {
    angle_max_out = scan_in.angle_max - scan_in.angle_increment*(mod_angles - 1);
    last_element = original_length-mod_angles+1; 
  } 
  else {
    angle_max_out = scan_in.angle_max - scan_in.angle_increment*(reduce_factor_ - 1); 
    last_element = original_length-reduce_factor_+1;
  }


  float angle_inc_out = scan_in.angle_increment * reduce_factor_;
  
  std::vector<float> ranges_out;
  std::vector<float> intensities_out;

  int j = 0;
  for (int i = 0; i <= last_element; i += reduce_factor_) {
    ranges_out[j] = scan_in.ranges[i];
    intensities_out[j] = scan_in.intensities[i];
    ++j;
  }

  scan_out.angle_max = angle_max_out;

  scan_out.ranges.resize(reduced_length);
  scan_out.ranges = ranges_out;
  
  scan_out.intensities.resize(reduced_length);
  scan_out.intensities = intensities_out;

  scan_out.angle_increment = angle_inc_out;

  ROS_INFO("Filtered out %d points from the laser scan", original_length-reduced_length);

  return true;
}
  