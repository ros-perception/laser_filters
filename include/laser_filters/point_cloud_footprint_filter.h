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

// #include <laser_geometry/laser_geometry.hpp>
#include <filters/filter_base.hpp>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

namespace laser_filters
{

class PointCloudFootprintFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud>
{
public:
  PointCloudFootprintFilter() {
    RCLCPP_WARN(logging_interface_->get_logger(), "PointCloudFootprintFilter has been deprecated.  Please use PR2PointCloudFootprintFilter instead.\n");
  }

  bool configure()
  {
    node_ = std::make_shared<rclcpp::Node>(getName());
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    if(!getParam("inscribed_radius", inscribed_radius_))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "PointCloudFootprintFilter needs inscribed_radius to be set");
      return false;
    }
    return true;
  }

  virtual ~PointCloudFootprintFilter()
  { 

  }

  bool update(const sensor_msgs::msg::PointCloud& input_scan, sensor_msgs::msg::PointCloud& filtered_scan)
  {
    if(&input_scan == &filtered_scan){
      RCLCPP_ERROR(logging_interface_->get_logger(), "This filter does not currently support in place copying");
      return false;
    }
    sensor_msgs::msg::PointCloud laser_cloud;

    try{
      geometry_msgs::msg::TransformStamped transform = buffer_->lookupTransform("base_link", input_scan.header.frame_id,
                     tf2::TimePointZero);
      tf2::doTransform(input_scan, laser_cloud, transform);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_ERROR(logging_interface_->get_logger(), "Transform unavailable %s", ex.what());
      return false;
    }  

    filtered_scan.header = input_scan.header;
    filtered_scan.points.resize (input_scan.points.size());
    filtered_scan.channels.resize (input_scan.channels.size());
    for (unsigned int d = 0; d < input_scan.channels.size (); d++){
      filtered_scan.channels[d].values.resize  (input_scan.points.size());
      filtered_scan.channels[d].name = input_scan.channels[d].name;
    }

    int num_pts = 0;
    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      if (!inFootprint(laser_cloud.points[i])){
        filtered_scan.points[num_pts] = input_scan.points[i];
        for (unsigned int d = 0; d < filtered_scan.channels.size (); d++)
          filtered_scan.channels[d].values[num_pts] = input_scan.channels[d].values[i];
        num_pts++;
      }
    }

    // Resize output vectors
    filtered_scan.points.resize (num_pts);
    for (unsigned int d = 0; d < filtered_scan.channels.size (); d++)
      filtered_scan.channels[d].values.resize (num_pts);

    return true;
  }


  bool inFootprint(const geometry_msgs::msg::Point32& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  double inscribed_radius_;
} ;

}

#endif // LASER_SCAN_FOOTPRINT_FILTER_H
