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


#ifndef LASER_FILTERS__BOX_FILTER_HPP_
#define LASER_FILTERS__BOX_FILTER_HPP_

#include <filters/filter_base.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>

typedef tf2::Vector3 Point;

#include <laser_geometry/laser_geometry.hpp>

#include <string>

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanBoxFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanBoxFilter();
  bool configure();

  bool update(
    const sensor_msgs::msg::LaserScan & input_scan,
    sensor_msgs::msg::LaserScan & filtered_scan);

private:
  bool inBox(Point & point);
  std::string box_frame_;
  laser_geometry::LaserProjection projector_;

  // tf listener to transform scans into the box_frame
  tf2_ros::TransformListener tf_;
  // A clock to use for time and sleeping
  rclcpp::Clock::SharedPtr clock;
  tf2_ros::Buffer buffer_;

  // defines two opposite corners of the box
  Point min_, max_;
  bool up_and_running_;

  rclcpp::Logger laser_filters_logger = rclcpp::get_logger("laser_filters");
};

}  // namespace laser_filters

#endif  // LASER_FILTERS__BOX_FILTER_HPP_
