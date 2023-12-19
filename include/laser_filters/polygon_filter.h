/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by Eurotec B.V.
 *  Copyright (c) 2020, Eurotec B.V.
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
 *  polygon_filter.h
 */

#ifndef POLYGON_FILTER_H
#define POLYGON_FILTER_H

#include <filters/filter_base.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

typedef tf2::Vector3 Point;
namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a polygon.
 */
class LaserScanPolygonFilterBase : public filters::FilterBase<sensor_msgs::msg::LaserScan> {
public:
  virtual bool configure();
  // virtual void configure(PolygonFilterConfig& config) { reconfigureCB(); }

  virtual bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) { return false; }

protected:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  boost::recursive_mutex own_mutex_;
  // configuration
  std::string polygon_frame_;
  geometry_msgs::msg::Polygon polygon_;
  double polygon_padding_;
  bool invert_filter_;
  bool is_polygon_published_ = false;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  virtual rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters);

  // checks if points in polygon
  bool inPolygon(const Point& point) const;

  void publishPolygon();
};

class LaserScanPolygonFilter : public LaserScanPolygonFilterBase {
public:
  bool configure() override;
  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override;

private:
  laser_geometry::LaserProjection projector_;
  // tf listener to transform scans into the polygon_frame
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};

/**
 * @brief This is a filter that removes points in a laser scan inside of a polygon.
 * It assumes that the transform between the scanner and the robot base remains unchanged,
 * i.e. the position and orientation of the laser filter should not change.
 * A typical use case for this filter is to filter out parts of the robot body or load that it may carry.
 */
class StaticLaserScanPolygonFilter : public LaserScanPolygonFilterBase {
public:
  bool configure() override;
  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override;
  
protected:
  bool transformPolygon(const std::string &input_scan_frame_id);
  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters) override;

private:
  double transform_timeout_;

  Eigen::ArrayXXd co_sine_map_;
  float co_sine_map_angle_min_;
  float co_sine_map_angle_max_;
  bool is_polygon_transformed_;

  void checkCoSineMap(const sensor_msgs::msg::LaserScan& input_scan);
  geometry_msgs::msg::PointStamped createPointStamped(const double &x, 
                                                      const double &y,
                                                      const double &z,
                                                      const builtin_interfaces::msg::Time &stamp,
                                                      const std::string &frame_id);
};
}
#endif /* polygon_filter.h */
