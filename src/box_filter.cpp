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

#include <memory>
#include <string>
#include <limits>

#include "laser_filters/box_filter.hpp"

laser_filters::LaserScanBoxFilter::LaserScanBoxFilter()
: clock(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  buffer_(clock), tf_(buffer_)
{
}

bool laser_filters::LaserScanBoxFilter::configure()
{
  up_and_running_ = true;
  double min_x, min_y, min_z, max_x, max_y, max_z;

  // Get the parameter value.
  bool box_frame_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("box_frame",
      box_frame_);
  bool x_max_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("max_x", max_x);
  bool y_max_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("max_y", max_y);
  bool z_max_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("max_z", max_z);
  bool x_min_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("min_x", min_x);
  bool y_min_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("min_y", min_y);
  bool z_min_set = FilterBase<sensor_msgs::msg::LaserScan>::node_->get_parameter("min_z", min_z);

  max_.setX(max_x);
  max_.setY(max_y);
  max_.setZ(max_z);
  min_.setX(min_x);
  min_.setY(min_y);
  min_.setZ(min_z);

  if (!box_frame_set) {
    RCLCPP_ERROR(laser_filters_logger, "box_frame is not set!");
  }
  if (!x_max_set) {
    RCLCPP_ERROR(laser_filters_logger, "max_x is not set!");
  }
  if (!y_max_set) {
    RCLCPP_ERROR(laser_filters_logger, "max_y is not set!");
  }
  if (!z_max_set) {
    RCLCPP_ERROR(laser_filters_logger, "max_z is not set!");
  }
  if (!x_min_set) {
    RCLCPP_ERROR(laser_filters_logger, "min_x is not set!");
  }
  if (!y_min_set) {
    RCLCPP_ERROR(laser_filters_logger, "min_y is not set!");
  }
  if (!z_min_set) {
    RCLCPP_ERROR(laser_filters_logger, "min_z is not set!");
  }

  return box_frame_set && x_max_set && y_max_set && z_max_set &&
         x_min_set && y_min_set && z_min_set;
}

bool laser_filters::LaserScanBoxFilter::update(
  const sensor_msgs::msg::LaserScan & input_scan,
  sensor_msgs::msg::LaserScan & output_scan)
{
  output_scan = input_scan;
  sensor_msgs::msg::PointCloud2 laser_cloud;

  std::string error_msg;

#ifndef TRANSFORM_LISTENER_NOT_IMPLEMENTED
  bool success = tf_.waitForTransform(
    box_frame_,
    input_scan.header.frame_id,
    input_scan.header.stamp +
    ros::Duration().fromSec(input_scan.ranges.size() * input_scan.time_increment),
    ros::Duration(1.0),
    ros::Duration(0.01),
    &error_msg
  );
  if (!success) {
    RCLCPP_WARN(laser_filters_logger, "Could not get transform, irgnoring laser scan! %s",
      error_msg.c_str());
    return false;
  }
#endif  // !TRANSFORM_LISTENER_NOT_IMPLEMENTED

  try {
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, buffer_);
  } catch (tf2::TransformException & ex) {
    if (up_and_running_) {
      RCLCPP_WARN(laser_filters_logger, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    } else {
      RCLCPP_INFO(laser_filters_logger, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }

#ifdef SENSOR_MSGS_POINT_CLOUD_CONVERSION_H
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1) {
    RCLCPP_INFO(laser_filters_logger, "x, y, z and index fields are required, skipping scan");
  }


  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const int64 pcount = laser_cloud.width * laser_cloud.height;
  const int64 limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;
  for (
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {
    // TODO(Rohit): works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78
    float x = *(reinterpret_cast<float *>(&laser_cloud.data[x_idx]));
    float y = *(reinterpret_cast<float *>(&laser_cloud.data[y_idx]));
    float z = *(reinterpret_cast<float *>(&laser_cloud.data[z_idx]));
    int index = *(reinterpret_cast<int *>(&laser_cloud.data[i_idx]));

    Point point(x, y, z);

    if (inBox(point)) {
      output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
    }
  }
#endif  // SENSOR_MSGS_POINT_CLOUD_CONVERSION_H
  up_and_running_ = true;
  return true;
}

bool laser_filters::LaserScanBoxFilter::inBox(Point & point)
{
  return
    point.x() < max_.x() && point.x() > min_.x() &&
    point.y() < max_.y() && point.y() > min_.y() &&
    point.z() < max_.z() && point.z() > min_.z();
}
