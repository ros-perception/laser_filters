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
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_FOOTPRINT_FILTER_H
#define LASER_SCAN_FOOTPRINT_FILTER_H
/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.
This is useful for ground plane extraction

**/

#include "filters/filter_base.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // PointCloud2ConstIterator
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "laser_geometry/laser_geometry.hpp"

namespace laser_filters
{

class LaserScanFootprintFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>, public rclcpp_lifecycle::LifecycleNode
{
public:
  LaserScanFootprintFilter()
      : rclcpp_lifecycle::LifecycleNode("laser_scan_footprint_filter"), 
        buffer_(get_clock()), tf_(buffer_), up_and_running_(false) {}

  bool configure()
  {
    if(!getParam("inscribed_radius", inscribed_radius_))
    {
      RCLCPP_ERROR(get_logger(), "LaserScanFootprintFilter needs inscribed_radius to be set");
      return false;
    }
    return true;
  }

  virtual ~LaserScanFootprintFilter()
  {
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan ;
    sensor_msgs::msg::PointCloud2 laser_cloud;

    try{
      projector_.transformLaserScanToPointCloud("base_link", input_scan, laser_cloud, buffer_);
    }
    catch (tf2::TransformException &ex)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      if (up_and_running_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1, "Dropping Scan: Transform unavailable %s", ex.what());
      }
      else
      {
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, .3, "Ignoring Scan: Waiting for TF");
      }
      return false;
    }

    sensor_msgs::PointCloud2ConstIterator<int> iter_i(laser_cloud, "index");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_cloud, "z");

    if (!(iter_i != iter_i.end()))
    {
      RCLCPP_ERROR(get_logger(), "We need an index channel to be able to filter out the footprint");
      return false;
    }

    for (;
         iter_x != iter_x.end() &&
         iter_y != iter_y.end() &&
         iter_z != iter_z.end() &&
         iter_i != iter_i.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {

      geometry_msgs::msg::Point32 point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;

      if (inFootprint(point))
      {
        filtered_scan.ranges[*iter_i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    up_and_running_ = true;
    return true;
  }

  bool inFootprint(const geometry_msgs::msg::Point32 &scan_pt)
  {
    if (scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
  bool up_and_running_;
} ;

}

#endif // LASER_SCAN_FOOTPRINT_FILTER_H
