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

#include <mutex>

#include <filters/filter_base.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


typedef tf2::Vector3 Point;
using std::placeholders::_1;

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

void padPolygon(geometry_msgs::msg::Polygon& polygon, double padding)
{
  // pad polygon in place
  for (unsigned int i = 0; i < polygon.points.size(); i++)
  {
    geometry_msgs::msg::Point32& pt = polygon.points[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return)
{  // Source: https://github.com/ros-planning/navigation/blob/melodic-devel/costmap_2d/src/array_parser.cpp
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
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
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        error_return = "More close ] than open [";
        return result;
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
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
        return result;
      }
      float value;
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
  }
  else
  {
    error_return = "";
  }

  return result;
}

geometry_msgs::msg::Polygon makePolygonFromString(const std::string& polygon_string, const geometry_msgs::msg::Polygon& last_polygon)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(polygon_string, error);

    if (error != "")
    {
      return last_polygon;
    }

    geometry_msgs::msg::Polygon polygon;
    geometry_msgs::msg::Point32 point;

    // convert vvf into points.
    if (vvf.size() < 3 && vvf.size() > 0)
    {
      return last_polygon;
    }

    for (unsigned int i = 0; i < vvf.size(); i++)
    {
      if (vvf[ i ].size() == 2)
      {
        point.x = vvf[ i ][ 0 ];
        point.y = vvf[ i ][ 1 ];
        point.z = 0;
        polygon.points.push_back(point);
      }
      else
      {
        return last_polygon;
      }
    }

    return polygon;
}

std::string polygonToString(geometry_msgs::msg::Polygon polygon)
{
  std::string polygon_string = "[";
  bool first = true;
  for (auto point : polygon.points) {
    if (!first) {
      polygon_string += ", ";
    }
    first = false;
    polygon_string += "[" + std::to_string(point.x) + ", " + std::to_string(point.y) + "]";
  }
  polygon_string += "]";
  return polygon_string;
}

static inline int getPointCloud2FieldIndex(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & field_name)
{
  // Get the index we need
  for (size_t d = 0; d < cloud.fields.size(); ++d) {
    if (cloud.fields[d].name == field_name) {
      return static_cast<int>(d);
    }
  }
  return -1;
}

using namespace std::literals;
namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a polygon.
 */
class LaserScanPolygonFilterBase : public filters::FilterBase<sensor_msgs::msg::LaserScan>, public rclcpp_lifecycle::LifecycleNode {
public:

  LaserScanPolygonFilterBase() : rclcpp_lifecycle::LifecycleNode("laser_scan_polygon_filter"), buffer_(get_clock()), tf_(buffer_){};

  virtual bool configure()
  {
    // dynamic reconfigure parameters callback:
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
              std::bind(&LaserScanPolygonFilterBase::reconfigureCB, this, std::placeholders::_1));

    std::string polygon_string;
    if(!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("footprint_topic"), footprint_topic_, false, "base_footprint_exclude"))
    {
      RCLCPP_WARN(logging_interface_->get_logger(), "Footprint topic not set, assuming default: base_footprint_exclude");
    }
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("polygon"), polygon_string))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: PolygonFilter was not given polygon.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("polygon_frame"), polygon_frame_, false))
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Error: PolygonFilter was not given polygon_frame.\n");
      return false;
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("invert"), invert_filter_, false))
    {
      RCLCPP_INFO(logging_interface_->get_logger(), "Error: PolygonFilter invert filter not set, assuming false.\n");
    }if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("polygon_padding"), polygon_padding_, false))
    {
      RCLCPP_INFO(logging_interface_->get_logger(), "Error: PolygonFilter polygon_padding not set, assuming 0. \n");
    }
    polygon_ = makePolygonFromString(polygon_string, polygon_);
    padPolygon(polygon_, polygon_padding_);
    
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("polygon", rclcpp::QoS(1).transient_local().keep_last(1));
    is_polygon_published_ = false;
    
    return true;
  }

  void footprintCB(const geometry_msgs::msg::Polygon::SharedPtr polygon)
  {
    if(polygon->points.size() < 3)
    {
      RCLCPP_WARN(logging_interface_->get_logger(), "Footprint needs at least three points for the robot polygon, ignoring message");
      return;
    }
    polygon_ = *polygon;
    padPolygon(polygon_, polygon_padding_);
    is_polygon_published_ = false;
  }
  virtual bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan) { return false; }

protected:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  std::recursive_mutex own_mutex_;
  // configuration
  std::string polygon_frame_;
  geometry_msgs::msg::Polygon polygon_;
  double polygon_padding_;
  bool invert_filter_;
  std::string footprint_topic_;
  bool is_polygon_published_ = false;
  

  // tf listener to transform scans into the right frame
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  virtual rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters)
  {
    std::lock_guard<std::recursive_mutex> lock(own_mutex_);
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      if(logging_interface_ != nullptr)
          RCLCPP_DEBUG_STREAM(logging_interface_->get_logger(), "Update parameter " << parameter.get_name().c_str()<< " to "<<parameter);
      if(parameter.get_name() == param_prefix_+"polygon"&& parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        std::string polygon_string = parameter.as_string();
        polygon_ = makePolygonFromString(polygon_string, polygon_);
      }
      else if(parameter.get_name() == param_prefix_+"polygon_frame" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        polygon_frame_ = parameter.as_string();
      }
      else if(parameter.get_name() == param_prefix_+"invert" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
        invert_filter_ = parameter.as_bool();
      }
      else if(parameter.get_name() == param_prefix_+"polygon_padding" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        polygon_padding_ = parameter.as_double();
      }
      else{
        RCLCPP_WARN(logging_interface_->get_logger(), "Unknown parameter");
      }
    }
    padPolygon(polygon_, polygon_padding_);
    is_polygon_published_ = false;
    return result;
  }

  // checks if points in polygon
  bool inPolygon(const Point& point) const
  {
     int i, j;
    bool c = false;

    for (i = 0, j = polygon_.points.size() - 1; i < polygon_.points.size(); j = i++)
    {
      if ((polygon_.points.at(i).y > point.y() != (polygon_.points.at(j).y > point.y()) &&
          (point.x() < (polygon_.points[j].x - polygon_.points[i].x) * (point.y() - polygon_.points[i].y) /
                                (polygon_.points[j].y - polygon_.points[i].y) +
                            polygon_.points[i].x)))
        c = !c;
    }
    return c;
  }

  void publishPolygon()
  {
    if (!is_polygon_published_)
    {
      geometry_msgs::msg::PolygonStamped polygon_stamped;
      polygon_stamped.header.frame_id = polygon_frame_;
      polygon_stamped.header.stamp = get_clock()->now();
      polygon_stamped.polygon = polygon_;
      polygon_pub_->publish(polygon_stamped);
      is_polygon_published_ = true;
    }
  }
};

class LaserScanPolygonFilter : public LaserScanPolygonFilterBase {
public:
  bool configure() override
  {
    bool result = LaserScanPolygonFilterBase::configure();
    footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(footprint_topic_, 1, std::bind(&LaserScanPolygonFilterBase::footprintCB, this, std::placeholders::_1));
    return result;
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan) override
  {
    auto start = std::chrono::high_resolution_clock::now();

    std::lock_guard<std::recursive_mutex> lock(own_mutex_);

    publishPolygon();

    output_scan = input_scan;

    sensor_msgs::msg::PointCloud2 laser_cloud;

    std::string error_msg;

    bool success = buffer_.canTransform(
      polygon_frame_,
      input_scan.header.frame_id,
      rclcpp::Time(input_scan.header.stamp) + std::chrono::duration<double>(input_scan.ranges.size() * input_scan.time_increment),
      1.0s, 
      &error_msg
    );
    if(!success){
      RCLCPP_WARN(logging_interface_->get_logger(), "Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
      return false;
    }

    try{
      projector_.transformLaserScanToPointCloud(polygon_frame_, input_scan, laser_cloud, buffer_);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_INFO_THROTTLE(logging_interface_->get_logger(), *get_clock(), 300, "Ignoring Scan: Waiting for TF");
      return false;
    }

    const int i_idx_c = getPointCloud2FieldIndex(laser_cloud, "index");
    const int x_idx_c = getPointCloud2FieldIndex(laser_cloud, "x");
    const int y_idx_c = getPointCloud2FieldIndex(laser_cloud, "y");
    const int z_idx_c = getPointCloud2FieldIndex(laser_cloud, "z");

    if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
    {
      RCLCPP_INFO_THROTTLE(logging_interface_->get_logger(), *get_clock(), 300, "x, y, z and index fields are required, skipping scan");
    }

    const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
    const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
    const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
    const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

    const int pstep = laser_cloud.point_step;
    const long int pcount = laser_cloud.width * laser_cloud.height;
    const long int limit = pstep * pcount;

    int i_idx, x_idx, y_idx, z_idx;
    for (i_idx = i_idx_offset, x_idx = x_idx_offset, y_idx = y_idx_offset, z_idx = z_idx_offset;

        x_idx < limit;

        i_idx += pstep, x_idx += pstep, y_idx += pstep, z_idx += pstep)
    {
      // TODO works only for float data types and with an index field
      // I'm working on it, see https://github.com/ros/common_msgs/pull/78
      float x = *((float*)(&laser_cloud.data[x_idx]));
      float y = *((float*)(&laser_cloud.data[y_idx]));
      float z = *((float*)(&laser_cloud.data[z_idx]));
      int index = *((int*)(&laser_cloud.data[i_idx]));

      Point point(x, y, z);

      if (!invert_filter_)
      {
        if (inPolygon(point))
        {
          output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
        }
      }
      else
      {
        if (!inPolygon(point))
        {
          output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto update_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_DEBUG(logging_interface_->get_logger(), "LaserScanPolygonFilter update took %lu microseconds", update_elapsed);

    return true;
  }

private:
  laser_geometry::LaserProjection projector_;
};

/**
 * @brief This is a filter that removes points in a laser scan inside of a polygon.
 * It assumes that the transform between the scanner and the robot base remains unchanged,
 * i.e. the position and orientation of the laser filter should not change.
 * A typical use case for this filter is to filter out parts of the robot body or load that it may carry.
 */
class StaticLaserScanPolygonFilter : public LaserScanPolygonFilterBase {
public:
  bool configure() override
  {
    bool result = LaserScanPolygonFilterBase::configure();
    is_polygon_transformed_ = false;
    transform_timeout_ = 5; // Default
    if (!filters::FilterBase<sensor_msgs::msg::LaserScan>::getParam(std::string("transform_timeout"), transform_timeout_))
    {
      RCLCPP_INFO(logging_interface_->get_logger(), "Error: PolygonFilter transform_timeout not set, assuming 5. \n");
    }
    footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(footprint_topic_, 1, std::bind(&StaticLaserScanPolygonFilter::footprintCB, this, std::placeholders::_1));
    return result;
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& output_scan) override
  {
    std::lock_guard<std::recursive_mutex> lock(own_mutex_);
    publishPolygon();

    if (!is_polygon_transformed_) 
    {
      if (!transformPolygon(input_scan.header.frame_id)) return false;
    }

    output_scan = input_scan;
    checkCoSineMap(input_scan);

    size_t i = 0;
    size_t i_max = input_scan.ranges.size();

    while (i < i_max)
    {
      float range = input_scan.ranges[i];

      float x = co_sine_map_(i, 0) * range;
      float y = co_sine_map_(i, 1) * range;
      Point point(x, y, 0);

      if (invert_filter_ != inPolygon(point))
      {
        output_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }

      ++i;
    }

    return true;
    }

  void footprintCB(const geometry_msgs::msg::Polygon::SharedPtr polygon)
  {
    is_polygon_transformed_ = false;
    LaserScanPolygonFilterBase::footprintCB(polygon);
  }
  
protected:
  bool transformPolygon(const std::string &input_scan_frame_id)
  {
    std::string error_msg;
    RCLCPP_DEBUG(logging_interface_->get_logger(),
      "waitForTransform %s -> %s",
      polygon_frame_.c_str(), input_scan_frame_id.c_str()
    );
    
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = buffer_.lookupTransform(input_scan_frame_id,
        polygon_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_));
    }
    catch(tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(logging_interface_->get_logger(),
          *get_clock(), 1000,
          "Could not get transform, ignoring laser scan! %s", ex.what());
          return false;
    }

    RCLCPP_INFO(logging_interface_->get_logger(), "Obtained transform");
    for (int i = 0; i < polygon_.points.size(); ++i)
    {
      geometry_msgs::msg::PointStamped point_in = createPointStamped(polygon_.points[i].x, polygon_.points[i].y, 0, transform.header.stamp, polygon_frame_);
      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, transform);
      polygon_.points[i].x = point_out.point.x;
      polygon_.points[i].y = point_out.point.y;
    }
    is_polygon_transformed_ = true;
    return true;
  }
  
  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters) override
  {
    is_polygon_transformed_ = false;
    return LaserScanPolygonFilterBase::reconfigureCB(parameters);
  }

private:
  double transform_timeout_;
  Eigen::ArrayXXd co_sine_map_;
  float co_sine_map_angle_min_;
  float co_sine_map_angle_max_;
  bool is_polygon_transformed_;

  void checkCoSineMap(const sensor_msgs::msg::LaserScan& scan_in)
  {
      size_t n_pts = scan_in.ranges.size();

    if (
      co_sine_map_.rows() != (int)n_pts ||
      co_sine_map_angle_min_ != scan_in.angle_min ||
      co_sine_map_angle_max_ != scan_in.angle_max
    ) {
      RCLCPP_DEBUG(logging_interface_->get_logger(), "No precomputed map given. Computing one.");
      co_sine_map_ = Eigen::ArrayXXd(n_pts, 2);
      co_sine_map_angle_min_ = scan_in.angle_min;
      co_sine_map_angle_max_ = scan_in.angle_max;

      // Spherical->Cartesian projection
      for (size_t i = 0; i < n_pts; ++i)
      {
        co_sine_map_(i, 0) = cos(scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map_(i, 1) = sin(scan_in.angle_min + (double) i * scan_in.angle_increment);
      }
    }
  }
  geometry_msgs::msg::PointStamped createPointStamped(const double &x, 
                                                      const double &y,
                                                      const double &z,
                                                      const builtin_interfaces::msg::Time &stamp,
                                                      const std::string &frame_id)
  {
    geometry_msgs::msg::PointStamped point;
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    point.header.stamp = stamp;
    point.header.frame_id = frame_id;
    return point;
  }
};
}
#endif /* polygon_filter.h */
