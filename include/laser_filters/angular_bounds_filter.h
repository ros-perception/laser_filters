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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef LASER_SCAN_ANGULAR_BOUNDS_FILTER_H
#define LASER_SCAN_ANGULAR_BOUNDS_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{
  class LaserScanAngularBoundsFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:
      double lower_angle_;
      double upper_angle_;

      bool configure()
      {
        lower_angle_ = 0;
        upper_angle_ = 0;

        if(!getParam("lower_angle", lower_angle_) || !getParam("upper_angle", upper_angle_)){
          ROS_ERROR("Both the lower_angle and upper_angle parameters must be set to use this filter.");
          return false;
        }

        return true;
      }

      virtual ~LaserScanAngularBoundsFilter(){}

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan){
        filtered_scan.ranges.resize(input_scan.ranges.size());
        filtered_scan.intensities.resize(input_scan.intensities.size());

        double start_angle = input_scan.angle_min;
        double current_angle = input_scan.angle_min;
        ros::Time start_time = input_scan.header.stamp;
        unsigned int count = 0;
        //loop through the scan and truncate the beginning and the end of the scan as necessary
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
          //wait until we get to our desired starting angle
          if(start_angle < lower_angle_){
            start_angle += input_scan.angle_increment;
            current_angle += input_scan.angle_increment;
            start_time += ros::Duration(input_scan.time_increment);
          }
          else{
            filtered_scan.ranges[count] = input_scan.ranges[i];

            //make sure  that we don't update intensity data if its not available
            if(input_scan.intensities.size() > i)
              filtered_scan.intensities[count] = input_scan.intensities[i];

            count++;

            //check if we need to break out of the loop, basically if the next increment will put us over the threshold
            if(current_angle + input_scan.angle_increment > upper_angle_){
              break;
            }

            current_angle += input_scan.angle_increment;

          }
        }

        //make sure to set all the needed fields on the filtered scan
        filtered_scan.header.frame_id = input_scan.header.frame_id;
        filtered_scan.header.stamp = start_time;
        filtered_scan.angle_min = start_angle;
        filtered_scan.angle_max = current_angle;
        filtered_scan.angle_increment = input_scan.angle_increment;
        filtered_scan.time_increment = input_scan.time_increment;
        filtered_scan.scan_time = input_scan.scan_time;
        filtered_scan.range_min = input_scan.range_min;
        filtered_scan.range_max = input_scan.range_max;

        filtered_scan.ranges.resize(count);

        if(input_scan.intensities.size() >= count)
          filtered_scan.intensities.resize(count);

        ROS_DEBUG("Filtered out %d points from the laser scan.", (int)input_scan.ranges.size() - (int)count);

        return true;

      }
  };
};
#endif
