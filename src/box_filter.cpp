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
 *  box_filter.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "laser_filters/box_filter.h"

namespace laser_filters
{

LaserScanBoxFilter::LaserScanBoxFilter(){

}

bool LaserScanBoxFilter::configure(){
  node_ = std::make_shared<rclcpp::Node>(getName());
  // dynamic reconfigure parameters callback:
  on_set_parameters_callback_handle_ = params_interface_->add_on_set_parameters_callback(
            std::bind(&LaserScanBoxFilter::reconfigureCB, this, std::placeholders::_1));

  buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  up_and_running_ = true;

  double min_x, min_y, min_z, max_x, max_y, max_z;
  bool invert = false;
  if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("box_frame"), box_frame_))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given box_frame.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_x"), max_x))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given max_x.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_y"), max_y))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given max_y.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("max_z"), max_z))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given max_z.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_x"), min_x))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given min_x.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_y"), min_y))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given min_y.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("min_z"), min_z))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given min_z.\n");
    return false;
  }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("invert"), invert))
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Error: BoxFilter was not given invert.\n");
    return false;
  }
  remove_box_points_ = not invert;
  
  max_.setX(max_x);
  max_.setY(max_y);
  max_.setZ(max_z);
  min_.setX(min_x);
  min_.setY(min_y);
  min_.setZ(min_z);

  RCLCPP_INFO(logging_interface_->get_logger(), "BOX filter started");
  RCLCPP_INFO(logging_interface_->get_logger(), "Box frame is: %s", box_frame_.c_str());
  RCLCPP_INFO(logging_interface_->get_logger(), "Box: x_min %f, x_max, %f, y_min, %f, y_max, %f, min_z %f, max_z %f", min_.getX(),
   max_.getX(), min_.getY(), max_.getY(), min_.getZ(), max_.getZ());
  RCLCPP_INFO(logging_interface_->get_logger(), "Box filter invert: %d", !remove_box_points_);
  return true;
}

bool LaserScanBoxFilter::update(
    const sensor_msgs::msg::LaserScan& input_scan,
    sensor_msgs::msg::LaserScan &output_scan)
{
  output_scan = input_scan;
  sensor_msgs::msg::PointCloud2 laser_cloud;
  
  std::string error_msg;

  bool success = buffer_->canTransform(
    box_frame_,
    input_scan.header.frame_id,
    rclcpp::Time(input_scan.header.stamp) + std::chrono::duration<double>(input_scan.ranges.size() * input_scan.time_increment),
    1.0s,
    &error_msg
  );
  if(!success){
    RCLCPP_WARN(logging_interface_->get_logger(), "Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
    return false;
  }

  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  try{
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, *buffer_);
  }
  catch(tf2::TransformException& ex){
    if(up_and_running_){
      RCLCPP_WARN_THROTTLE(logging_interface_->get_logger(), steady_clock, 1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      RCLCPP_INFO_THROTTLE(logging_interface_->get_logger(), steady_clock, .3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<int> iter_i(laser_cloud, "index");
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_cloud, "z");      
      
  if (
    !(iter_i != iter_i.end()) || 
    !(iter_x != iter_x.end()) || 
    !(iter_y != iter_y.end()) || 
    !(iter_z != iter_z.end()))
  {
    RCLCPP_INFO_THROTTLE(logging_interface_->get_logger(), steady_clock, .3, "x, y, z and index fields are required, skipping scan");
  }

  for (;
      iter_x != iter_x.end() &&
      iter_y != iter_y.end() &&
      iter_z != iter_z.end() &&
      iter_i != iter_i.end();
      ++iter_x, ++iter_y, ++iter_z, ++iter_i)
  {
    Point point(*iter_x, *iter_y, *iter_z);

    if (remove_box_points_ == inBox(point))
    {
      output_scan.ranges[*iter_i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  up_and_running_ = true;
  return true;
}

bool LaserScanBoxFilter::inBox(Point &point){
  return point.x() < max_.x() && point.x() > min_.x() &&
          point.y() < max_.y() && point.y() > min_.y() &&
          point.z() < max_.z() && point.z() > min_.z();
}

rcl_interfaces::msg::SetParametersResult LaserScanBoxFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == "box_frame"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
          box_frame_ = parameter.as_string();
      else if(parameter.get_name() == "invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
           bool invert = parameter.as_bool();
           remove_box_points_ = not invert;
      }
      else if(parameter.get_name() == "max_x" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_.setX(parameter.as_double());
      else if(parameter.get_name() == "max_y" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_.setY(parameter.as_double());
      else if(parameter.get_name() == "max_z" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          max_.setZ(parameter.as_double());
      else if(parameter.get_name() == "min_x" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          min_.setX(parameter.as_double());
      else if(parameter.get_name() == "min_y" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          min_.setY(parameter.as_double());
      else if(parameter.get_name() == "min_z" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          min_.setZ(parameter.as_double());
      else
        RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
    }

  return result;
}

} //namespace laser_filters
