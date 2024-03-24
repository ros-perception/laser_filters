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
 *  polygon_filter.cpp
 */

#include "laser_filters/polygon_filter.h"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <laser_filters/PolygonFilterConfig.h>
#include <geometry_msgs/PolygonStamped.h>
#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

void padPolygon(geometry_msgs::Polygon& polygon, double padding)
{
  // pad polygon in place
  for (unsigned int i = 0; i < polygon.points.size(); i++)
  {
    geometry_msgs::Point32& pt = polygon.points[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the polygon specification (param %s) must be numbers. Found value %s.",
              full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the polygon specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

geometry_msgs::Polygon makePolygonFromXMLRPC(const XmlRpc::XmlRpcValue& polygon_xmlrpc,
                                             const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (polygon_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      polygon_xmlrpc.size() > 0 && polygon_xmlrpc.size() < 3)
  {
    ROS_FATAL("The polygon (parameter %s) must be specified as nested list on the parameter server with at least "
              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]",
              full_param_name.c_str());

    throw std::runtime_error("The polygon must be specified as nested list on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 pt;

  for (int i = 0; i < polygon_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = polygon_xmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2)
    {
      ROS_FATAL("The polygon (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                full_param_name.c_str());
      throw std::runtime_error("The polygon must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[0], full_param_name);
    pt.y = getNumberFromXMLRPC(point[1], full_param_name);

    polygon.points.push_back(pt);
  }
  return polygon;
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

geometry_msgs::Polygon makePolygonFromString(const std::string& polygon_string, const geometry_msgs::Polygon& last_polygon)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(polygon_string, error);

    if (error != "")
    {
      ROS_ERROR("Error parsing polygon parameter: '%s'", error.c_str());
      ROS_ERROR(" Polygon string was '%s'.", polygon_string.c_str());
      return last_polygon;
    }

    geometry_msgs::Polygon polygon;
    geometry_msgs::Point32 point;

    // convert vvf into points.
    if (vvf.size() < 3 && vvf.size() > 0)
    {
      ROS_WARN("You must specify at least three points for the robot polygon");
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
        ROS_ERROR("Points in the polygon specification must be pairs of numbers. Found a point with %d numbers.",
                   int(vvf[ i ].size()));
        return last_polygon;
      }
    }

    return polygon;
}

std::string polygonToString(geometry_msgs::Polygon polygon)
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

namespace laser_filters{

bool LaserScanPolygonFilterBase::configure()
{
  XmlRpc::XmlRpcValue polygon_xmlrpc;
  std::string polygon_string;
  PolygonFilterConfig param_config;

  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::PolygonFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<laser_filters::PolygonFilterConfig>::CallbackType f;
  f = [this](auto& config, auto level){ reconfigureCB(config, level); };

  bool polygon_set = getParam("polygon", polygon_xmlrpc);
  bool polygon_frame_set = getParam("polygon_frame", polygon_frame_);
  bool invert_set = getParam("invert", invert_filter_);
  polygon_ = makePolygonFromXMLRPC(polygon_xmlrpc, "polygon");

  double polygon_padding = 0;
  getParam("polygon_padding", polygon_padding);

  polygon_string = polygonToString(polygon_);
  param_config.polygon = polygon_string;
  param_config.polygon_padding = polygon_padding;
  param_config.invert = invert_filter_;
  dyn_server_->updateConfig(param_config);

  // Calling setCallback(f) here calls reconfigureCB() which updates the polygon padding.
  dyn_server_->setCallback(f);

  polygon_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);
  is_polygon_published_ = false;

  if (!polygon_frame_set)
  {
    ROS_ERROR("polygon_frame is not set!");
  }
  if (!polygon_set)
  {
    ROS_ERROR("polygon is not set!");
  }
  if (!invert_set)
  {
    ROS_INFO("invert filter not set, assuming false");
    invert_filter_ = false;
  }

  return polygon_frame_set && polygon_set;
}

// See https://web.cs.ucdavis.edu/~okreylos/TAship/Spring2000/PointInPolygon.html
bool LaserScanPolygonFilterBase::inPolygon(tf::Point& point) const {
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

void LaserScanPolygonFilterBase::publishPolygon()
{
  if (!is_polygon_published_)
  {
    geometry_msgs::PolygonStamped polygon_stamped;
    polygon_stamped.header.frame_id = polygon_frame_;
    polygon_stamped.header.stamp = ros::Time::now();
    polygon_stamped.polygon = polygon_;
    polygon_pub_.publish(polygon_stamped);
    is_polygon_published_ = true;
  }
}

void LaserScanPolygonFilterBase::reconfigureCB(laser_filters::PolygonFilterConfig& config, uint32_t level)
{
  invert_filter_ = config.invert;
  polygon_ = makePolygonFromString(config.polygon, polygon_);
  padPolygon(polygon_, config.polygon_padding);
  is_polygon_published_ = false;
}

bool LaserScanPolygonFilter::update(const sensor_msgs::LaserScan& input_scan,
                                    sensor_msgs::LaserScan& output_scan)
{
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  publishPolygon();

  output_scan = input_scan;

  sensor_msgs::PointCloud2 laser_cloud;

  std::string error_msg;

  bool success = tf_.waitForTransform(
      polygon_frame_, input_scan.header.frame_id,
      input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size() * input_scan.time_increment),
      ros::Duration(1.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform, ignoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try
  {
    projector_.transformLaserScanToPointCloud(polygon_frame_, input_scan, laser_cloud, tf_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
  {
    ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");
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

    tf::Point point(x, y, z);

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

  return true;
}

bool StaticLaserScanPolygonFilter::configure()
{
  is_polygon_transformed_ = false;

  transform_timeout_ = 5; // Default
  getParam("transform_timeout", transform_timeout_);

  return LaserScanPolygonFilterBase::configure();
}

void StaticLaserScanPolygonFilter::checkCoSineMap(const sensor_msgs::LaserScan& scan_in)
{
  size_t n_pts = scan_in.ranges.size();

  if (
    co_sine_map_.rows() != (int)n_pts ||
    co_sine_map_angle_min_ != scan_in.angle_min ||
    co_sine_map_angle_max_ != scan_in.angle_max
  ) {
    ROS_DEBUG_NAMED("StaticLaserScanPolygonFilter", "No precomputed map given. Computing one.");
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

// Note: This implementation transforms the polygon relative to the laser.
// It does this lazily and only once. This has the advantage that the check if points fall inside the polygon is fast
// as the transform is not needed there. Furthermore, it means that the filter chain node
// does not need to be continuously subscribed to the transform topic, which significantly reduces CPU load.
// A pre-requisite for this to work is that the transform is static, i.e. the position and orientation of the laser with regard to
// the base of the robot does not change.
bool StaticLaserScanPolygonFilter::update(const sensor_msgs::LaserScan& input_scan,
                                          sensor_msgs::LaserScan& output_scan)
{
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  publishPolygon();

  if (!is_polygon_transformed_) {
    tf::TransformListener transform_listener;

    std::string error_msg;
    ROS_DEBUG_NAMED(
      "StaticLaserScanPolygonFilter", "waitForTransform %s -> %s",
      polygon_frame_.c_str(), input_scan.header.frame_id.c_str()
    );
    bool success = transform_listener.waitForTransform(
      input_scan.header.frame_id, polygon_frame_,
      ros::Time(),       // No restrictions on transform time. It is static.
      ros::Duration(transform_timeout_),
      ros::Duration(0),  // This setting has no effect
      &error_msg
    );

    if (!success)
    {
      ROS_WARN_THROTTLE_NAMED(
          1, "StaticLaserScanPolygonFilter",
          "Could not get transform, ignoring laser scan! %s", error_msg.c_str()
      );
      return false;
    }
    else
    {
      ROS_INFO_NAMED("StaticLaserScanPolygonFilter", "Obtained transform");
    }

    try {
      // Transform each point of polygon. This includes multiple type convertions because of transformPoint API requiring Stamped<Point>
      // which does not in turn expose coordinate values
      for (int i = 0; i < polygon_.points.size(); ++i)
      {
        tf::Point point(polygon_.points[i].x, polygon_.points[i].y, 0);
        tf::Stamped<tf::Point> point_stamped(point, ros::Time(), polygon_frame_);
        tf::Stamped<tf::Point> point_stamped_new;
        transform_listener.transformPoint(input_scan.header.frame_id, point_stamped, point_stamped_new);
        geometry_msgs::PointStamped result_point;
        tf::pointStampedTFToMsg(point_stamped_new, result_point);
        polygon_.points[i].x = result_point.point.x;
        polygon_.points[i].y = result_point.point.y;
      }

      is_polygon_transformed_ = true;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1, "StaticLaserScanPolygonFilter", "Exception while transforming polygon");
      return false;
    }
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
    tf::Point point(x, y, 0);

    if (invert_filter_ != inPolygon(point))
    {
      output_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }

    ++i;
  }

  return true;
}

void StaticLaserScanPolygonFilter::reconfigureCB(laser_filters::PolygonFilterConfig& config, uint32_t level)
{
  is_polygon_transformed_ = false;
  LaserScanPolygonFilterBase::reconfigureCB(config, level);
}
}
