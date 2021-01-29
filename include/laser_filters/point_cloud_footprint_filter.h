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

#ifndef POINT_CLOUD_FOOTPRINT_FILTER_H
#define POINT_CLOUD_FOOTPRINT_FILTER_H
/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.  
This is useful for ground plane extraction

**/

#include "laser_geometry/laser_geometry.hpp"
#include "filters/filter_base.hpp"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
//#include "pcl_ros/transforms.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace laser_filters
{

class PointCloudFootprintFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>, public rclcpp_lifecycle::LifecycleNode
{
public:
  PointCloudFootprintFilter() : rclcpp_lifecycle::LifecycleNode("point_cloud_footprint_filter"), buffer_(get_clock()), tf_(buffer_)
  {
    RCLCPP_WARN(get_logger(), "PointCloudFootprintFilter has been deprecated.  Please use PR2PointCloudFootprintFilter instead.\n");
  }

  bool configure()
  {
    if(!getParam("inscribed_radius", inscribed_radius_))
    {
      RCLCPP_ERROR(get_logger(), "PointCloudFootprintFilter needs inscribed_radius to be set");
      return false;
    }
    return true;
  }

  virtual ~PointCloudFootprintFilter()
  { 

  }

  bool update(const sensor_msgs::msg::PointCloud2 &input_scan, sensor_msgs::msg::PointCloud2 &filtered_scan)
  {
    if (&input_scan == &filtered_scan)
    {
      RCLCPP_ERROR(get_logger(), "This filter does not currently support in place copying");
      return false;
    }
    sensor_msgs::msg::PointCloud2 laser_cloud;

    try
    {
//      pcl_ros::transformPointCloud("base_link", input_scan, laser_cloud, buffer_);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(get_logger(), "Transform unavailable %s", ex.what());
      return false;
    }

    filtered_scan.header = laser_cloud.header;
    filtered_scan.height = 1;
    // filtered_scan.width = 0;
    filtered_scan.fields = laser_cloud.fields;
    filtered_scan.is_bigendian = laser_cloud.is_bigendian;
    filtered_scan.point_step = laser_cloud.point_step;
    // filtered_scan.row_step = 0;
    filtered_scan.data.reserve(laser_cloud.data.size());
    filtered_scan.is_dense = laser_cloud.is_dense;

    int index = 0;
    auto iter_x = sensor_msgs::PointCloud2ConstIterator<float>(laser_cloud, "x");
    for (; iter_x != iter_x.end(); ++index, ++iter_x)
    {
      geometry_msgs::msg::Point32 point;
      point.x = (&*iter_x)[0];
      point.y = (&*iter_x)[1];
      point.z = (&*iter_x)[2];

      if (inFootprint(point))
      {
        filtered_scan.data.insert(
            filtered_scan.data.end(),
            laser_cloud.data.begin() + (index + 0) * laser_cloud.point_step,
            laser_cloud.data.begin() + (index + 1) * laser_cloud.point_step);
      }
      filtered_scan.row_step = filtered_scan.data.size();
      filtered_scan.width = filtered_scan.row_step / filtered_scan.point_step;
    }

      return true;
  }

  bool inFootprint(const geometry_msgs::msg::Point32& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
} ;

}

#endif // LASER_SCAN_FOOTPRINT_FILTER_H
