/*
 * Copyright (c) 2020 Nicolas Limpert <limpert@fh-aachen.de>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
  \author Nicolas Limpert <limpert@fh-aachen.de>


*/

#ifndef LASER_SCAN_RADIUS_OUTLIER_FILTER_H
#define LASER_SCAN_RADIUS_OUTLIER_FILTER_H

#include <set>

#include <filters/filter_base.h>
#include <laser_filters/radius_outlier_detector.h>
#include <laser_filters/RadiusOutlierFilterConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>
#include <math.h>


namespace laser_filters
{
/** @b RadiusOutlierFilter filters scan points that don't have at least
 *     some number of neighbors within a certain range.
 */

class RadiusOutlierFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  double radius_search_;
  int window_, min_neighbors_;

  RadiusOutlierDetector radius_detector_;

  std::shared_ptr<dynamic_reconfigure::Server<laser_filters::RadiusOutlierFilterConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;
  RadiusOutlierFilterConfig param_config;

  RadiusOutlierFilter()
  {
  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    ros::NodeHandle private_nh("~" + getName());
    dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::RadiusOutlierFilterConfig>(own_mutex_, private_nh));
    dynamic_reconfigure::Server<laser_filters::RadiusOutlierFilterConfig>::CallbackType f;
    f = boost::bind(&laser_filters::RadiusOutlierFilter::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(f);

    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("radius_search"), radius_search_))
    {
      ROS_ERROR("Error: RadiusOutlierFilter was not given radius_search.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("min_neighbors"), min_neighbors_))
    {
      ROS_ERROR("Error: RadiusOutlierFilter was not given min_neighbors.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("window"), window_))
    {
      ROS_ERROR("Error: RadiusOutlierFilter was not given window.\n");
      return false;
    }

    if (radius_search_ < 0)
    {
      ROS_ERROR("radius_search must be >= 0. Forcing radius_search = 0.");
      radius_search_ = 0.0;
    }

    if (min_neighbors_ < 0)
    {
      ROS_ERROR("min_neighbors must be >= 0. Forcing min_neighbors = 0");
      min_neighbors_ = 0;
    }
    radius_detector_.configure(radius_search_);

    param_config.radius_search = radius_search_;
    param_config.window = window_;
    param_config.min_neighbors = min_neighbors_;
    dyn_server_->updateConfig(param_config);

    return true;
  }

  void reconfigureCB(RadiusOutlierFilterConfig& config, uint32_t level)
  {
    radius_search_ = config.radius_search;
    window_ = config.window;
    min_neighbors_ = config.min_neighbors;

    radius_detector_.configure(radius_search_);
  }

  virtual ~RadiusOutlierFilter()
  {
  }

  /** \brief Filter points based on 3 global parameters: radius_search, min_neighbors and
   * window. The radius_search specifies the maximum allowed distance for
   * points to be neighbors. The min_neighbors specifies how many neighboring points
   * have to be found (within radius_search being the maximum allowed distance) to consider
   * the point to be valid. The window parameter specifies how many consecutive measurements
   * to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
  {
    boost::recursive_mutex::scoped_lock lock(own_mutex_);
    // copy across all data first
    scan_out = scan_in;

    std::set<int> indices_to_delete;
    // For each point in the current line scan
    for (int i = 0; i < static_cast<int>(scan_in.ranges.size()); i++)
    {
      int num_neighbors = 0;

      // Look around the current point until either the window is exceeded
      // or the number of neighbors was found.
      for (int y = -window_; y < window_ + 1 && num_neighbors < min_neighbors_; y++)
      {
        int j = i + y;

        if (j < 0 || j >= static_cast<int>(scan_in.ranges.size()) || i == j || std::isnan(scan_in.ranges[j]))
        {  // Out of scan bounds or itself or infinity
          continue;
        }

        if (radius_detector_.isNeighbor(scan_in.ranges[i], scan_in.ranges[j], y * scan_in.angle_increment))
        {
          num_neighbors++;
        }
      }

      if (num_neighbors < min_neighbors_)
      {
        indices_to_delete.insert(i);
      }
    }

    ROS_DEBUG("RadiusOutlierFilter removing %d/%d Points from scan",
              static_cast<int>(indices_to_delete.size()), static_cast<int>(scan_in.ranges.size()));
    for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
    {
      scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();  // Failed test to set the ranges to invalid value
    }
    return true;
  }

};
}

#endif  // LASER_SCAN_RADIUS_OUTLIER_FILTER_H
