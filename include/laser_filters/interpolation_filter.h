/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#ifndef INTERPOLATION_FILTER_H
#define INTERPOLATION_FILTER_H
/**
\author Eitan Marder-Eppstein
@b InterpolationFilter takes input scans and for readings that come back as errors, interpolates between valid readings to generate range values for them.

**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class InterpolationFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  bool configure()
  {
    return true;
  }

  virtual ~InterpolationFilter()
  { 
  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    double previous_valid_range = input_scan.range_max - .01;
    double next_valid_range = input_scan.range_max - .01;
    filtered_scan= input_scan;

    unsigned int i = 0;
    while(i < input_scan.ranges.size()) // Need to check every reading in the current scan
    {
      //check if the reading is out of range for some reason
      if (filtered_scan.ranges[i] <= input_scan.range_min ||
          filtered_scan.ranges[i] >= input_scan.range_max){

        //we need to find the next valid range reading
        unsigned int j = i + 1;
        unsigned int start_index = i;
        unsigned int end_index = i;
        while(j < input_scan.ranges.size()){
          if (filtered_scan.ranges[j] <= input_scan.range_min || 
              filtered_scan.ranges[j] >= input_scan.range_max){                                                                                 
            end_index = j;
          }
          else{
            next_valid_range = filtered_scan.ranges[j];
            break;
          }
          ++j;
        }

        //for now, we'll just take the average between the two valid range readings
        double average_range = (previous_valid_range + next_valid_range) / 2.0;

        for(unsigned int k = start_index; k <= end_index; k++){
          filtered_scan.ranges[k] = average_range;
        }

        //make sure to update our previous valid range reading
        previous_valid_range = next_valid_range;
        i = j + 1;
      }
      else{
        previous_valid_range = filtered_scan.ranges[i];
        ++i;
      }

    }
    return true;
  }
};

}

#endif // LASER_SCAN_INTENSITY_FILTER_H
