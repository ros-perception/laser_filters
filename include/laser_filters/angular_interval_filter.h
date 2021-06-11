/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, JSK (The University of Tokyo).
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

#ifndef LASER_SCAN_ANGULAR_INTERVAL_FILTER_H
#define LASER_SCAN_ANGULAR_INTERVAL_FILTER_H
/**
\author Ulas Sureyya Bingol
@b ScanAngularIntervalFilter takes input scans and filters the readings within the ranges of interval sets.
**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include <laser_filters/AngularIntervalFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace laser_filters
{

class LaserScanAngularIntervalFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  std::string interval_sets_;
  std::vector<std::vector<double>> intervals_;
  std::shared_ptr<dynamic_reconfigure::Server<laser_filters::AngularIntervalFilterConfig>> dyn_server_;
  AngularIntervalFilterConfig param_config;
  boost::recursive_mutex own_mutex_;

  bool enable_ = true;
  bool got_intervals_ = false;
  bool configure()
  {
    ros::NodeHandle private_nh("~" + getName());
    dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::AngularIntervalFilterConfig>(own_mutex_, private_nh));
    dynamic_reconfigure::Server<laser_filters::AngularIntervalFilterConfig>::CallbackType f;
    f = boost::bind(&laser_filters::LaserScanAngularIntervalFilter::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(f);


    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("interval_sets"), interval_sets_))
    {
      ROS_WARN("AngularIntervalFilter was not given interval sets.\n");
    }
    else
    {
      param_config.interval_sets = interval_sets_;
      dyn_server_->updateConfig(param_config);
    }
    return true;
  }

  virtual ~LaserScanAngularIntervalFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;
    double replacement_value = std::numeric_limits<double>::quiet_NaN();
    double current_angle = input_scan.angle_min;
    if(got_intervals_ && enable_)
    {
      for (unsigned int i = 0; i < input_scan.ranges.size(); i++) // Need to check ever reading in the current scan
      {
        current_angle = input_scan.angle_min + i* input_scan.angle_increment;
        for(auto interval : intervals_)
        {
          if(current_angle < interval[1]  && current_angle > interval[0])
            filtered_scan.ranges[i] = replacement_value;
        }
      }
    }
    return true;
  }
  
  void reconfigureCB(AngularIntervalFilterConfig& config, uint32_t level)
  {
  	interval_sets_ = config.interval_sets; // th11,th12;th21,th22;th31,th32
  	enable_ = config.enable;

    std::string text = interval_sets_;
    std::string parse_error;
    if(parseIntervalString(text, parse_error, intervals_))
    {
      ROS_INFO("Got intervals %s", parse_error.c_str());
      got_intervals_ = true;
    }
    else
    {
      got_intervals_ = false;
      ROS_WARN("Failed to calculate intervals: %s", parse_error.c_str());
    }
  }

  static bool compare2dVect(std::vector<double> v1, std::vector<double> v2) 
  { 
    return (v1[0] < v2[0]); 
  } 


  bool parseIntervalString(const std::string& input, std::string& error_return, std::vector<std::vector<double>>& interval)
  {
    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<double> current_vector;
    interval.clear();
    while (!!input_ss && !input_ss.eof())
    {
      switch (input_ss.peek())
      {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2)
        {
          error_return = "Array depth greater than 2";
          return false;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0)
        {
          error_return = "More close ] than open [";
          return false;
        }
        input_ss.get();
        if (depth == 1)
        {
          if(current_vector.size() == 2)
            interval.push_back(current_vector);
          else
          {
            error_return = "Sizes of the interval isn't correct. There has to be two elements";
            return false;
          }
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2)
        {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return false;
        }
        double value;
        input_ss >> value;
        if (!!input_ss)
        {
          current_vector.push_back(value);
        }
        break;
      }
    }

    if (depth != 0)
    {
      error_return = "Unterminated vector string.";
      return false;
    }
    else
    {
      error_return = "";
    }
    for(auto range : interval)
    {
      std::string debug_text = "Interval:";
      for(double val : range)
      {
        debug_text = debug_text + std::to_string(val) + ",";
      }
      ROS_INFO("Val %s", debug_text.c_str());
    }

    return true;
  }

};

}

#endif // LASER_SCAN_RANGE_INTERVAL_FILTER_H
