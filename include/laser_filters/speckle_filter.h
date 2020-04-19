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
 *  speckle_filter.h
 */

#ifndef SPECKLE_FILTER_H
#define SPECKLE_FILTER_H

#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <laser_filters/SpeckleFilterConfig.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{

class WindowValidator
{
public:
  virtual bool checkWindowValid(const sensor_msgs::LaserScan& scan, size_t idx, size_t window, double max_range_difference) = 0;
};

class DistanceWindowValidator : public WindowValidator
{
  virtual bool checkWindowValid(const sensor_msgs::LaserScan& scan, size_t idx, size_t window, double max_range_difference)
  {
    const float& range = scan.ranges[idx];
    if (range != range) {
      return false;
    }

    for (size_t neighbor_idx_nr = 1; neighbor_idx_nr < window; ++neighbor_idx_nr)
    {
      size_t neighbor_idx = idx + neighbor_idx_nr;
      if (neighbor_idx < scan.ranges.size())  // Out of bound check
      {
        const float& neighbor_range = scan.ranges[neighbor_idx];
        if (neighbor_range != neighbor_range || fabs(neighbor_range - range) > max_range_difference)
        {
          return false;
        }
      }
    }
    return true;
  }
};

class RadiusOutlierWindowValidator : public WindowValidator
{
  virtual bool checkWindowValid(const sensor_msgs::LaserScan& scan, size_t idx, size_t window, double max_distance)
  {
    int num_neighbors = 0;
    const float& r1 = scan.ranges[idx];
    float r2 = 0.;

    // Look around the current point until either the window is exceeded
    // or the number of neighbors was found.
    for (int y = -(int)window; y < (int)window + 1 && num_neighbors < (int)window; y++)
    {
      int j = idx + y;
      r2 = scan.ranges[j];

      if (j < 0 || j >= static_cast<int>(scan.ranges.size()) || idx == j || std::isnan(r2))
      {  // Out of scan bounds or itself or infinity
        continue;
      }

      // Explanation:
      //
      // Distance between two points:
      // d² = (x2 - x1)² + (y2 - y1)²
      //
      // Substitute x with r * cos(phi) and y with r * sin(phi):
      // d² = (r2 * cos(phi2) - r1 * cos(phi1))² + (r2 * sin(phi2) - r1 * sin(phi1))²
      //
      // Apply binomial theorem:
      // d² = ((r2² * cos(phi2)² + r1² * cos(phi1)² - 2 * r1 * r2 * cos(phi1) * cos(phi2)) +
      //      ((r2² * sin(phi2)² + r1² * sin(phi1)² - 2 * r1 * r2 * sin(phi1) * sin(phi2))
      //
      // Merge sums:
      // d² = r2² * (cos(phi2)² + sin(phi2)²) + r1² * (cos(phi1)² + sin(phi1)² -
      //      2 * r1 * r2 * (cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2))
      //
      // Apply cos² + sin² = 1:
      // d² = r2² + r1² - 2 * r1 * r2 * (cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2))
      //
      // Note the following:
      // cos(phi1) * cos(phi2) = 1/2 * (cos(phi1 - phi2) + cos(phi1 + phi2))
      // sin(phi1) * sin(phi2) = 1/2 * (cos(phi1 - phi2) - cos(phi1 + phi2))
      //
      // cos(phi1) * cos(phi2) + sin(phi1) * sin(phi2) = cos(phi1 - phi2)
      //
      // Finally, phi1 - phi2 is our included_angle.

      const float d = sqrt(
            pow(r1,2) + pow(r2,2) -
            (2 * r1 * r2 * cosf(y * scan.angle_increment)));


      if (d <= max_distance)
      {
        num_neighbors++;
      }
    }

    // consider the window to be the number of neighbors we need
    if (num_neighbors < window)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
};

/**
 * @brief This is a filter that removes speckle points in a laser scan based on consecutive ranges
 */
class LaserScanSpeckleFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  LaserScanSpeckleFilter();
  ~LaserScanSpeckleFilter();
  bool configure();
  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan);

private:
  std::shared_ptr<dynamic_reconfigure::Server<laser_filters::SpeckleFilterConfig>> dyn_server_;
  void reconfigureCB(laser_filters::SpeckleFilterConfig& config, uint32_t level);
  boost::recursive_mutex own_mutex_;

  SpeckleFilterConfig config_ = SpeckleFilterConfig::__getDefault__();
  WindowValidator* validator_;
};
}
#endif /* speckle_filter.h */
