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
 *  box_filter.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */



#ifndef BOXFILTER_H
#define BOXFILTER_H

#include <filters/filter_base.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>

typedef tf2::Vector3 Point;

#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanBoxFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>, public rclcpp_lifecycle::LifecycleNode
{
  public:
    LaserScanBoxFilter() : rclcpp_lifecycle::LifecycleNode("laser_scan_box_filter"), buffer_(get_clock()), tf_(buffer_){};

    bool configure()
    {
      up_and_running_ = true;
      double min_x, min_y, min_z, max_x, max_y, max_z;
      bool box_frame_set = getParam("box_frame", box_frame_);
      bool x_max_set = getParam("max_x", max_x);
      bool y_max_set = getParam("max_y", max_y);
      bool z_max_set = getParam("max_z", max_z);
      bool x_min_set = getParam("min_x", min_x);
      bool y_min_set = getParam("min_y", min_y);
      bool z_min_set = getParam("min_z", min_z);

      max_.setX(max_x);
      max_.setY(max_y);
      max_.setZ(max_z);
      min_.setX(min_x);
      min_.setY(min_y);
      min_.setZ(min_z);

      bool invert = false;
      getParam("invert", invert);
      remove_box_points_ = !invert;

      if (!box_frame_set)
      {
        RCLCPP_ERROR(get_logger(), "box_frame is not set!");
      }
      if (!x_max_set)
      {
        RCLCPP_ERROR(get_logger(), "max_x is not set!");
      }
      if (!y_max_set)
      {
        RCLCPP_ERROR(get_logger(), "max_y is not set!");
      }
      if (!z_max_set)
      {
        RCLCPP_ERROR(get_logger(), "max_z is not set!");
      }
      if (!x_min_set)
      {
        RCLCPP_ERROR(get_logger(), "min_x is not set!");
      }
      if (!y_min_set)
      {
        RCLCPP_ERROR(get_logger(), "min_y is not set!");
      }
      if (!z_min_set)
      {
        RCLCPP_ERROR(get_logger(), "min_z is not set!");
      }

      return box_frame_set && x_max_set && y_max_set && z_max_set &&
             x_min_set && y_min_set && z_min_set;
    }

    bool update(
        const sensor_msgs::msg::LaserScan &input_scan,
        sensor_msgs::msg::LaserScan &output_scan)
    {
      using namespace std::chrono_literals;
      output_scan = input_scan;
      sensor_msgs::msg::PointCloud2 laser_cloud;

      std::string error_msg;

      bool success = buffer_.canTransform(
          box_frame_,
          input_scan.header.frame_id,
          rclcpp::Time(input_scan.header.stamp) + std::chrono::duration<double>(input_scan.ranges.size() * input_scan.time_increment),
          1.0s,
          &error_msg);
      if (!success)
      {
        RCLCPP_WARN(get_logger(), "Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
        return false;
      }

      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      try
      {
        projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, buffer_);
      }
      catch (tf2::TransformException &ex)
      {
        if (up_and_running_)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1, "Dropping Scan: Tansform unavailable %s", ex.what());
          return true;
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
      
      if (
        !(iter_i != iter_i.end()) || 
        !(iter_x != iter_x.end()) || 
        !(iter_y != iter_y.end()) || 
        !(iter_z != iter_z.end()))
      {
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, .3, "x, y, z and index fields are required, skipping scan");
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

  private:
    bool inBox(Point &point)
    {
      return point.x() < max_.x() && point.x() > min_.x() &&
             point.y() < max_.y() && point.y() > min_.y() &&
             point.z() < max_.z() && point.z() > min_.z();
    }
    std::string box_frame_;
    laser_geometry::LaserProjection projector_;

    // tf listener to transform scans into the box_frame
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_;

    // parameter to decide if points in box or points outside of box are removed
    bool remove_box_points_ = true;

    // defines two opposite corners of the box
    Point min_, max_;
    bool up_and_running_;
  };

} // namespace laser_filters

#endif /* box_filter.h */
